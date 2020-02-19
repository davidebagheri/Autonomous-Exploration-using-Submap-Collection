# Submap-autonomous-exploration

Motion planner for the autonous exploration of unknokn environment with drones. 

It is based on Voxgraph, a mapping framework that represents the space as a set of submaps, aligned with a pose optimization
in order to lower the robot drift over time.

The planner is instead made of two sub-planners: a Receding Horizon Next-Best-View one for local exploration, and a global
Frontier planner that acts when the local planner is stuck or the invormative gain is low.

![Screenshot from 2020-02-16 12-34-11](https://user-images.githubusercontent.com/30367721/74822108-e67abf00-5304-11ea-985c-2f2ed6e76c9e.png)

