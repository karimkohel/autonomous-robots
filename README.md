# ROSBOT CONTROLLER

### Contributors
- Kareem Kohel
- Mohamed Badawy
- Mohamed Ithar

## Structure

Every folder contains a `rosbot_controller` folder, this is the important folder.

Each maze folder holds the controller that can be used to solve that maze. in this instance the maze 5 & 4 folder has one controller that is generalized to solve both mazes, so it should be treated as a controller for each maze.

```
.
├── ARCHITECTURE_NOTES.md
├── Atonomous_Robots_document.pdf
├── maze_3
│   └── rosbot_controller
│       ├── config.py
│       ├── mission_runtime.py
│       ├── occupancy_grid.py
│       ├── robot_io.py
│       ├── rosbot_controller.py
│       └── utils.py
├── maze_5&4
│   └── rosbot_controller
│       ├── config.py
│       ├── mission_runtime.py
│       ├── occupancy_grid.py
│       ├── robot_io.py
│       ├── rosbot_controller_maze4.py
│       └── utils.py
├── README.md
└── videos
    ├── maze3.mp4
    ├── maze4.mp4
    └── Maze5.mp4
```
## Running the code

To run the controller for any maze.

copy the `rosbot_controller` folder from inside the maze_# folder and place it inside the controller folder of the maze you want to start.

### for example.

#### Getting the right code

If you want to test our code against maze 5. then head on to our project and copy the `rosbot_controller` folder that is inside the `maze_5&4` directory.

then go to the maze 5 folder that contains the actual maze, go to the controllers directory and past the folder there. 

#### running the world

start the webots world you want to test, for this example we chose 5. 

make sure the simulation is stopped or restart the world and stop it if it was running.

navigate to the `rosbot` in the right side scene tree and expand it. 
then click on controller and instead of rosbot, chooose the `rosbot_controller` controller from the list of available worlds

#### watch the robot go


just for clarification, we only were able to solve maps 3,4,5. using 1 main code base, and one modified scenario.


