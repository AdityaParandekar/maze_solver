## Autonomous maze solver

This package has 3 main scripts :

1. path_planner: RRT algorithm to compute the final path of the given maze 
2. controller : PID controller to move the bot between two points of the path_planner
3. color_detector: To detect and print the color at the final goal. 


Sample output of the path_planner node 

<p align="center">
    <img src="./src/assests/sample_path.png" width="389" height="324">  
</p>


Final Result on running:
> roslaunch maze_solver omnibase_maze.launch

<p align="center">
    <img src="./src/assests/final_run.png">
</p>