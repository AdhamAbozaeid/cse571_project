## Running the Demo

To start the demo, run the following commands in order:

1. Run `./env_setup.sh`
2. Run `roscore`
3. Run `rosrun cse571_project server.py -d 6 -n 40 -f 5 -b 5   `
4. Run `roslaunch cse571_project maze.launch`
5. Enter desired goal position in terminal running server.py (if running with -default-goal 0)
6. Run `rosrun cse571_project move_tbot3.py`
7. Run `rosrun cse571_project search.py -a astar -c`

### server.py

Implements action execution and environment updation functions

#### Command-line arguments for server.py:

  -h, --help            show this help message and exit
  -d 5                     for providing dimension of the grid
  -n 15                   for providing no. of obstacles to be added in the grid
  -s 32                    for providing random seed
  -f 3                       for providing no. of refueling stations
  -b 15                    for providing initial battery level
  -hdl 0                   for running in headless mode(0 or 1)
  -default-goal 0   for running with default goal 

**Note:**

- Enter desired goal position after running `roslaunch cse571_project maze.launch`
- For entering goal position, please use only values for x and y coordinates within the grid-dimension/2. For example, the acceptable values for x position of goal for a gird dimension of 4 are: 0,0.5,1,1.5,2

#### environment_api.py

Implements an interface to communicate with the server script.  Please modify the **state_to_check** and **action_to_check** variables in script as required for code inspection

#### mazeGenerator.py

Environment generation script

#### move_tbot3.py, pid_pro.py

Handles movement of turtlebot. pid_pro.py adds onto pid.py with integrative error compensation.