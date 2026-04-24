/*
 * esp_bot_firmware.ino
 * 
 * Hardware:
 *   ESP32-WROOM-32 (v1.1)
 *   2x N20 geared motors (300:1) with hall encoders
 *   DRV8833 dual H-bridge motor driver
 *   MPU9250 9-axis IMU (I2C)
 *   Grid cell: 150mm
 *
 * micro-ROS over WiFi (UDP)
 * Subscribes: /<BOT_ID>/cmd   (std_msgs/String)
 * Publishes:  /<BOT_ID>/status (std_msgs/String)
 *
 * Command protocol:
 *   "ROTATE:<target_heading_deg>:<seq>"
 *   "MOVE:<distance_mm>:<seq>"
 *   "STOP:0:<seq>"
 *
 * Status protocol:
 *   "<bot_id>:<status>:<seq>:<enc_left_mm>:<enc_right_mm>:<heading_deg>"
 *   status = IDLE | EXECUTING | DONE | ERROR
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <Wire.h>

// ═══════════════════════════════════════════════════════════════
// CONFIGURATION — CHANGE THESE PER BOT
// ═══════════════════════════════════════════════════════════════

// ── Identity ──
#define BOT_ID          "bot_01"     // Change to "bot_02" for second bot

// ── WiFi ──
#define WIFI_SSID       "your wifi ssid"
#define WIFI_PASS       "your wifi pwd"
#define AGENT_IP        "your laptop ip"  // Your laptop's IP
#define AGENT_PORT      8888

// ═══════════════════════════════════════════════════════════════
// PIN ASSIGNMENTS — ESP32-WROOM-32 + DRV8833 + N20 + MPU9250
// ═══════════════════════════════════════════════════════════════

// DRV8833 motor driver (PWM directly on input pins, no enable)
#define MOTOR_L_FWD     26    // AIN1 — Left motor forward
#define MOTOR_L_REV     25   // AIN2 — Left motor reverse
#define MOTOR_R_FWD     27   // BIN1 — Right motor forward
#define MOTOR_R_REV     14   // BIN2 — Right motor reverse
#define DRV_SLEEP       4    // nSLEEP — pull HIGH to enable driver

// N20 encoder pins (quadrature, 2 channels each)
#define ENC_L_A         18   // Left encoder channel A
#define ENC_L_B         19   // Left encoder channel B
#define ENC_R_A         32   // Right encoder channel A
#define ENC_R_B         33   // Right encoder channel B

// MPU9250 I2C
#define IMU_SDA         21
#define IMU_SCL         22
#define MPU_ADDR        0x68

// ═══════════════════════════════════════════════════════════════
// ROBOT PHYSICAL PARAMETERS
// ═══════════════════════════════════════════════════════════════

// N20 300:1 encoder: 7 PPR at motor shaft
// Counting RISING edge of channel A only = 7 * 300 = 2100 counts/rev
// If counting BOTH edges of channel A    = 14 * 300 = 4200 counts/rev
// We use RISING on channel A = 2100 counts/rev
#define ENCODER_CPR         3500

// Wheel — MEASURE YOUR ACTUAL WHEEL and update this
#define WHEEL_DIAMETER_MM   34.0
#define WHEEL_CIRCUMFERENCE (3.14159265 * WHEEL_DIAMETER_MM)  // ~106.8mm
#define MM_PER_COUNT        (WHEEL_CIRCUMFERENCE / ENCODER_CPR)  // ~0.0509mm

// Wheelbase — MEASURE center-to-center distance between wheels
#define WHEEL_BASE_MM       95.0

// Grid
#define GRID_SIZE_MM        150

// ═══════════════════════════════════════════════════════════════
// PID GAINS — TUNE THESE ON YOUR PHYSICAL BOT
// ═══════════════════════════════════════════════════════════════

// Rotation PID (error = degrees)
#define ROT_KP      4.0
#define ROT_KI      0.01
#define ROT_KD      1.5

// Straight-line PID (error = mm remaining)
#define MOVE_KP     1.0
#define MOVE_KI     0.000
#define MOVE_KD     0.0

// Heading correction during straight moves
#define HEADING_CORR_KP  0.0

// ═══════════════════════════════════════════════════════════════
// TOLERANCES & LIMITS
// ═══════════════════════════════════════════════════════════════

#define HEADING_TOL_DEG     3.0      // close enough for rotation
#define DISTANCE_TOL_MM     3.0      // close enough for move
#define PWM_MIN             50       // minimum PWM to overcome friction
#define PWM_MAX             150      // cap to prevent overshoot
#define COMMAND_TIMEOUT_MS  8000     // abort if command takes too long
#define PID_INTEGRAL_MAX    500.0    // anti-windup clamp

// PWM config
#define PWM_FREQ    5000
#define PWM_BITS    8    // 0-255

// ═══════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════

// Encoders (volatile — modified in ISR)
volatile long enc_left_count  = 0;
volatile long enc_right_count = 0;

// IMU
float heading_deg   = 0.0;    // current heading, 0-360
float gyro_z_offset = 0.0;    // calibrated bias

// Bot state machine
enum State { IDLE, ROTATING, MOVING, DONE_ST, ERROR_ST };
State bot_state = IDLE;

float  target_heading    = 0.0;
float  target_distance   = 0.0;
float  dist_traveled     = 0.0;
uint32_t cmd_seq         = 0;
unsigned long cmd_start  = 0;

// PID accumulators
float pid_i = 0.0;
float pid_prev_err = 0.0;

// micro-ROS handles
rcl_subscription_t  sub;
rcl_publisher_t     pub;
std_msgs__msg__String sub_msg;
std_msgs__msg__String pub_msg;
rclc_executor_t     executor;
rcl_allocator_t     allocator;
rclc_support_t      support;
rcl_node_t          node;
rcl_timer_t         timer;

char sub_buf[256];
char pub_buf[256];

// ═══════════════════════════════════════════════════════════════
// ENCODER ISRs
// ═══════════════════════════════════════════════════════════════

void IRAM_ATTR isr_enc_left() {
    // Read channel B to determine direction
    if (digitalRead(ENC_L_B))
        enc_left_count--;
    else
        enc_left_count++;
}

void IRAM_ATTR isr_enc_right() {
    if (digitalRead(ENC_R_B))
        enc_right_count--;
    else
        enc_right_count++;
}

// ═══════════════════════════════════════════════════════════════
// MPU9250
// ═══════════════════════════════════════════════════════════════

void imu_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

int16_t imu_read_16(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
    int16_t val = (Wire.read() << 8) | Wire.read();
    return val;
}

void imu_init() {
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);   // 400kHz I2C

    // Wake up MPU9250
    imu_write_reg(0x6B, 0x00);  // PWR_MGMT_1: wake up
    delay(100);
    imu_write_reg(0x6B, 0x01);  // Use PLL with X-axis gyro ref
    delay(10);

    // Gyro config: ±500°/s range (good balance of range vs precision)
    imu_write_reg(0x1B, 0x08);  // GYRO_CONFIG: FS_SEL = 1 (±500°/s)

    // Accel config: ±4g
    imu_write_reg(0x1C, 0x08);  // ACCEL_CONFIG: AFS_SEL = 1 (±4g)

    // Low-pass filter: 42Hz bandwidth (smooths noise, ~1ms latency)
    imu_write_reg(0x1A, 0x03);  // CONFIG: DLPF_CFG = 3

    // Sample rate divider: 1kHz / (1+4) = 200Hz
    imu_write_reg(0x19, 0x04);  // SMPLRT_DIV = 4

    delay(100);
}

void imu_calibrate_gyro() {
    /*
     * Calibrate gyro Z bias — bot must be STATIONARY during this.
     * Average 500 samples over ~2.5 seconds.
     */
    Serial.println("Calibrating gyro — keep bot still...");
    float sum = 0.0;
    int n = 500;
    for (int i = 0; i < n; i++) {
        int16_t raw = imu_read_16(0x47);  // GYRO_ZOUT_H
        sum += raw;
        delay(5);
    }
    gyro_z_offset = sum / n;
    Serial.print("Gyro Z offset: ");
    Serial.println(gyro_z_offset);
}

float imu_read_gyro_z_dps() {
    /*
     * Read gyroscope Z-axis in degrees/second.
     * MPU9250 at ±500°/s: sensitivity = 65.5 LSB/(°/s)
     */
    int16_t raw = imu_read_16(0x47);  // GYRO_ZOUT_H
    return (raw - gyro_z_offset) / 65.5;
}

void imu_update_heading(float dt) {
    float gz = imu_read_gyro_z_dps();

    // Dead zone: ignore tiny readings (noise)
    if (fabs(gz) < 0.5) gz = 0.0;

    heading_deg += gz * dt;

    // Normalize to 0-360
    while (heading_deg < 0.0)    heading_deg += 360.0;
    while (heading_deg >= 360.0) heading_deg -= 360.0;
}

// ═══════════════════════════════════════════════════════════════
// DRV8833 MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════
/*
 * DRV8833 truth table per motor:
 *   xIN1=PWM, xIN2=LOW  → Forward at PWM duty
 *   xIN1=LOW, xIN2=PWM  → Reverse at PWM duty
 *   xIN1=LOW, xIN2=LOW  → Coast (motor free)
 *   xIN1=HIGH,xIN2=HIGH → Brake (motor shorted)
 *
 * We use 4 LEDC channels, one per pin:
 *   Channel 0 = MOTOR_L_FWD (GPIO 25)
 *   Channel 1 = MOTOR_L_REV (GPIO 26)
 *   Channel 2 = MOTOR_R_FWD (GPIO 27)
 *   Channel 3 = MOTOR_R_REV (GPIO 14)
 */

void motors_init() {
    // Enable the driver
    pinMode(DRV_SLEEP, OUTPUT);
    digitalWrite(DRV_SLEEP, HIGH);

    // New ESP32 Arduino Core v3.x API:
    // ledcAttach(pin, freq, resolution_bits)
    ledcAttach(MOTOR_L_FWD, PWM_FREQ, PWM_BITS);
    ledcAttach(MOTOR_L_REV, PWM_FREQ, PWM_BITS);
    ledcAttach(MOTOR_R_FWD, PWM_FREQ, PWM_BITS);
    ledcAttach(MOTOR_R_REV, PWM_FREQ, PWM_BITS);

    stop_motors();
}

void set_motor_left(int speed) {
    if (speed > 0) {
        ledcWrite(MOTOR_L_FWD, constrain(speed, 0, 255));
        ledcWrite(MOTOR_L_REV, 0);
    } else if (speed < 0) {
        ledcWrite(MOTOR_L_FWD, 0);
        ledcWrite(MOTOR_L_REV, constrain(-speed, 0, 255));
    } else {
        ledcWrite(MOTOR_L_FWD, 0);
        ledcWrite(MOTOR_L_REV, 0);
    }
}

void set_motor_right(int speed) {
    if (speed > 0) {
        ledcWrite(MOTOR_R_FWD, constrain(speed, 0, 255));
        ledcWrite(MOTOR_R_REV, 0);
    } else if (speed < 0) {
        ledcWrite(MOTOR_R_FWD, 0);
        ledcWrite(MOTOR_R_REV, constrain(-speed, 0, 255));
    } else {
        ledcWrite(MOTOR_R_FWD, 0);
        ledcWrite(MOTOR_R_REV, 0);
    }
}

void set_motors(int left, int right) {
    set_motor_left(left);
    set_motor_right(right);
}

void stop_motors() {
    set_motors(0, 0);
}
void brake_motors() {
    ledcWrite(MOTOR_L_FWD, 255);
    ledcWrite(MOTOR_L_REV, 255);
    ledcWrite(MOTOR_R_FWD, 255);
    ledcWrite(MOTOR_R_REV, 255);
    delay(50);
    stop_motors();
}

// ═══════════════════════════════════════════════════════════════
// PID HELPERS
// ═══════════════════════════════════════════════════════════════

float angle_diff(float from_deg, float to_deg) {
    float d = fmod(to_deg - from_deg, 360.0);
    if (d > 180.0)  d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

void pid_reset() {
    pid_i = 0.0;
    pid_prev_err = 0.0;
}

float pid_compute(float error, float kp, float ki, float kd, float dt) {
    pid_i += error * dt;
    // Anti-windup
    if (pid_i >  PID_INTEGRAL_MAX) pid_i =  PID_INTEGRAL_MAX;
    if (pid_i < -PID_INTEGRAL_MAX) pid_i = -PID_INTEGRAL_MAX;

    float derivative = (dt > 0.001) ? (error - pid_prev_err) / dt : 0.0;
    pid_prev_err = error;

    return kp * error + ki * pid_i + kd * derivative;
}

// ═══════════════════════════════════════════════════════════════
// COMMAND EXECUTION
// ═══════════════════════════════════════════════════════════════

void execute_rotate(float dt) {
    // Timeout check
    if (millis() - cmd_start > COMMAND_TIMEOUT_MS) {
        brake_motors();
        bot_state = ERROR_ST;
        Serial.println("ROTATE timeout!");
        return;
    }

    float error = angle_diff(heading_deg, target_heading);

    if (fabs(error) < HEADING_TOL_DEG) {
        brake_motors();
        pid_reset();
        bot_state = DONE_ST;
        Serial.print("ROTATE done. Heading: ");
        Serial.println(heading_deg);
        return;
    }

    float output = pid_compute(error, ROT_KP, ROT_KI, ROT_KD, dt);
    int speed = constrain(abs((int)output), PWM_MIN, PWM_MAX);

    if (error > 0) {
        // Turn clockwise (when viewed from above):
        // left wheel forward, right wheel backward
        set_motors(speed, -speed);
    } else {
        set_motors(-speed, speed);
    }
}

void execute_move(float dt) {
    // Timeout check
    if (millis() - cmd_start > COMMAND_TIMEOUT_MS) {
        brake_motors();
        bot_state = ERROR_ST;
        Serial.println("MOVE timeout!");
        return;
    }

    // Distance from encoders (average of both wheels)
    float left_mm  = enc_left_count  * MM_PER_COUNT;
    float right_mm = enc_right_count * MM_PER_COUNT;
    dist_traveled = (left_mm + right_mm) / 2.0;

    float dist_error = target_distance - dist_traveled;

    if (dist_error < DISTANCE_TOL_MM) {
        brake_motors();
        pid_reset();
        bot_state = DONE_ST;
        Serial.print("MOVE done. Traveled: ");
        Serial.print(dist_traveled);
        Serial.println(" mm");
        return;
    }

    // Distance PID → base speed
    float speed_output = pid_compute(dist_error, MOVE_KP, MOVE_KI, MOVE_KD, dt);
    int base = constrain(abs((int)speed_output), PWM_MIN, PWM_MAX);

    // Heading correction to drive straight
    float h_error = angle_diff(heading_deg, target_heading);
    int correction = (int)(h_error * HEADING_CORR_KP);

    set_motors(base + correction, base - correction);
}

// ═══════════════════════════════════════════════════════════════
// MICRO-ROS COMMAND CALLBACK
// ═══════════════════════════════════════════════════════════════

void cmd_callback(const void *msg_in) {
    const std_msgs__msg__String *msg =
        (const std_msgs__msg__String *)msg_in;

     // IGNORE new commands if already executing one
    if (bot_state == ROTATING || bot_state == MOVING) {
        Serial.println("BUSY — ignoring command");
        return;
    }
    String cmd = String(msg->data.data);
    Serial.print("CMD: ");
    Serial.println(cmd);

    // Parse "TYPE:VALUE:SEQ"
    int sep1 = cmd.indexOf(':');
    int sep2 = cmd.indexOf(':', sep1 + 1);
    if (sep1 < 0 || sep2 < 0) {
        Serial.println("Bad command format");
        return;
    }

    String type  = cmd.substring(0, sep1);
    float  value = cmd.substring(sep1 + 1, sep2).toFloat();
    uint32_t seq = cmd.substring(sep2 + 1).toInt();

    cmd_seq   = seq;
    cmd_start = millis();
    pid_reset();

    if (type == "ROTATE") {
        target_heading = value;
        bot_state = ROTATING;
        Serial.print("  → Rotating to ");
        Serial.print(target_heading);
        Serial.print("° from ");
        Serial.print(heading_deg);
        Serial.println("°");

    } else if (type == "MOVE") {
        target_distance = value;
        target_heading  = heading_deg;  // lock current heading
        enc_left_count  = 0;
        enc_right_count = 0;
        dist_traveled   = 0.0;
        bot_state = MOVING;
        Serial.print("  → Moving ");
        Serial.print(target_distance);
        Serial.println(" mm");

    } else if (type == "STOP") {
        brake_motors();
        bot_state = IDLE;
        Serial.println("  → Stopped");
    }
}

// ═══════════════════════════════════════════════════════════════
// PUBLISH STATUS
// ═══════════════════════════════════════════════════════════════

void publish_status(const char *status_str) {
    float left_mm  = enc_left_count  * MM_PER_COUNT;
    float right_mm = enc_right_count * MM_PER_COUNT;

    snprintf(pub_buf, sizeof(pub_buf),
             "%s:%s:%lu:%.1f:%.1f:%.1f",
             BOT_ID, status_str, (unsigned long)cmd_seq,
             left_mm, right_mm, heading_deg);

    pub_msg.data.data = pub_buf;
    pub_msg.data.size = strlen(pub_buf);
    rcl_publish(&pub, &pub_msg, NULL);

    Serial.print("STATUS: ");
    Serial.println(pub_buf);
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
     pinMode(DRV_SLEEP, OUTPUT); 
    digitalWrite(DRV_SLEEP, LOW);  // driver OFF
    
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("================================");
    Serial.print("Swarm Bot: ");
    Serial.println(BOT_ID);
    Serial.println("================================");

    // ── Hardware init ─────────────────────────────────────────
    motors_init();
    Serial.println("[OK] Motors");

    // Encoder pins
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_enc_left,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_enc_right, RISING);
    Serial.println("[OK] Encoders");

    // IMU
    imu_init();
    Serial.println("[OK] MPU9250");
    imu_calibrate_gyro();
    Serial.println("[OK] Gyro calibrated");

    Serial.println("Connecting to WiFi...");
WiFi.begin(WIFI_SSID, WIFI_PASS);

// WAIT until we are actually connected
while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
}

Serial.println("\n[OK] WiFi Connected!");
Serial.print("IP Address: ");
Serial.println(WiFi.localIP());

// NOW initialize micro-ROS
set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);

    // ── micro-ROS init ────────────────────────────────────────
    Serial.println("Connecting to WiFi...");
    set_microros_wifi_transports(
        (char*)WIFI_SSID, (char*)WIFI_PASS,
        (char*)AGENT_IP, AGENT_PORT);
    delay(2000);

    allocator = rcl_get_default_allocator();

    rcl_ret_t ret;
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.println("[FAIL] rclc_support_init");
        while(1) delay(1000);
    }

    ret = rclc_node_init_default(&node, BOT_ID, "", &support);
    if (ret != RCL_RET_OK) {
        Serial.println("[FAIL] node init");
        while(1) delay(1000);
    }
    Serial.println("[OK] ROS2 node");

    // Subscribe: /<BOT_ID>/cmd
    char cmd_topic[64];
    snprintf(cmd_topic, sizeof(cmd_topic), "/%s/cmd", BOT_ID);
    ret = rclc_subscription_init_default(
        &sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        cmd_topic);
    if (ret != RCL_RET_OK) {
        Serial.print("[FAIL] subscribe ");
        Serial.println(cmd_topic);
    } else {
        Serial.print("[OK] Sub: ");
        Serial.println(cmd_topic);
    }

    // Publish: /<BOT_ID>/status
    char status_topic[64];
    snprintf(status_topic, sizeof(status_topic), "/%s/status", BOT_ID);
    ret = rclc_publisher_init_default(
        &pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        status_topic);
    if (ret != RCL_RET_OK) {
        Serial.print("[FAIL] publish ");
        Serial.println(status_topic);
    } else {
        Serial.print("[OK] Pub: ");
        Serial.println(status_topic);
    }

    // Executor (1 subscription)
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // Allocate message buffers
    sub_msg.data.data     = sub_buf;
    sub_msg.data.size     = 0;
    sub_msg.data.capacity = sizeof(sub_buf);

    pub_msg.data.data     = pub_buf;
    pub_msg.data.size     = 0;
    pub_msg.data.capacity = sizeof(pub_buf);

    rclc_executor_add_subscription(
        &executor, &sub, &sub_msg, &cmd_callback, ON_NEW_DATA);

    Serial.println("================================");
    Serial.println("Ready. Waiting for commands...");
    Serial.println("================================");
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP — runs at ~100Hz
// ═══════════════════════════════════════════════════════════════

unsigned long last_loop_us = 0;
unsigned long status_interval_ms = 0;

void loop() {
    // Spin micro-ROS (check for incoming messages)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

    // Compute dt
    unsigned long now_us = micros();
    float dt = (now_us - last_loop_us) / 1000000.0;
    if (dt < 0.008) return;  // ~120Hz max
    last_loop_us = now_us;

    // Update heading from IMU
    imu_update_heading(dt);

    // State machine
    switch (bot_state) {
        case ROTATING:
            execute_rotate(dt);
            break;

        case MOVING:
            execute_move(dt);
            break;

        case DONE_ST:
            publish_status("DONE");
            bot_state = IDLE;
            break;

        case ERROR_ST:
            publish_status("ERROR");
            stop_motors();
            bot_state = IDLE;
            break;

        case IDLE:
        default:
            // Publish heartbeat every 2 seconds
            if (millis() - status_interval_ms > 2000) {
                publish_status("IDLE");
                status_interval_ms = millis();
            }
            break;
    }
}
