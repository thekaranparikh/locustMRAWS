# Fastest Time Logic files breakdown

<ins>sim.py</ins> general ui for tinkering, no logic <br><br>

<ins>logic01.py</ins> is a single bot logic which follows the edges of the rectangle to push a block to a goal. no additional features.<br>
<ins>sim_logic01.py</ins> is used to simulate this 1 bot-block-goal logic. <br><br>

<ins>logic02.py</ins> is an updated version of on bot-block-goal system. Here the first spawn occurance of bot-block-goal are active, and everything spawned afterwards just acts as a obstacle.<br>
Features:<br>
- single bot block system with cost based path selection 
- bot take the wider corridor path if both paths have the same ticks 
- obstacle classification as : static not movable(other blocks), static movable(other bots), dynamic(other bot/block which are moving)(*not yet done)
- add blocks in the route as obstacle, it reroutes thorugh the obstacle / chooses other path(whichever has lowest tick cost)
- add bots in the route as obstacle (the bot moves out of the route)
- helper bot logic (we add the cost of other bot aswell but in ticks and not route*)
- allow the bots to wait also, if there is a helper bot setting up the block to a corridor from which the active bot will pick it up. not always will of bot look for reroutes to achive fastest time and waiting can also sometime give the fastest time.
- Keeping the helper bot as an helper only and not as an active bot(in multibot case the ideal bot which is an helper now, can ressign the task to itself it it sole takes a shorter path)
<ins>sim_logic02.py<ins> is used to simulate 1 bot-block-goal logic with features file.<br><br>

<ins>logic_multi.pt</ins> this is for multibot system, it uses the logic01.py file for now. (*This does not have any features yet) <br>
<ins>ui01.py</ins> is used to simulate the multibot swarm logic