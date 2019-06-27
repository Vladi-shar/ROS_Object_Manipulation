# Objects Manipulation

## Project Report

Alex Lavriv and Vladimir Shargorodsky

Advisor: Prof. Ronen Brafman

20.06.2019

The problem:

Pick up a desired object with the robot arm after clearing the way from obstacle objects.



Program description

The program controls the robotic arm such that it moves obstacle objects from an initial board state to a target board state.

In the target state the robotic arm can grip and pick up the desired object.



Program implementation

Tools:

ROS (Robot Operating System)

- --Gazebo

- --Manipulator-H library
- --C++
- --Python

Nodejs

- --JavaScript



Nodejs Server and gui client:

We represented the desired object and the obstacles in a matrix.

Using the web client you can insert the initial board state, the target state and send them to the server.

The server will find the set of instructions that will bring the board from the initial state to the target state (if possible), and send them to the client.



Mov\_boxes node in ROS:

The node will receive the instructions and perform each in order until the board is in the target state.

The arm will then move to the desired object, grab it and pick it up.



Main Files

**Mov\_box\_node.cpp**

Converts string instructions to robotic arm movement using the manipulator-h library.

functions_:_

_Perform\_insctruction_

- --Perform a single instruction e.g. _(1,4),left_.

The following functions move the head in alignment to the matrix cell:

_Mov\_head\_to\_y\_then\_x_

- --Move the arm head to specific y coordinate and then x.

_Mov\_head\_to\_x\_then\_y_

- --Move the arm head to specific x coordinate and then y.

These 4 functions move the **head** one cell at a specified direction:

_Mov\_forward_

_Mov\_backward_

_Mov\_left_

_Mov\_right_

<br>

_Mov\_fingers_

- --Moves the gripper&#39;s fingers to the desired position.

_waitForEndTrajectory_

- --Pauses execution of instructions until the previous instruction is completed.



**Main.py**

Receives an initial and target board states and returns a set of instructions required to transform the initial to the target state (if one exists)



**Server.js**

Provides a web server for the web client - using nodejs and express.









Running the code

In order to run the simulation in gazebo, run the following commands in terminal:



Launches the gazebo simulator with the arm and the gripper attached.



Launches the arm manipulator.



Moves the boxes according to the solution string.

For example:





**Optional** - Runs the manipulator-h gui that allows controlling the arm manually.

Provided are two world configurations to use in the gazebo simulation:

Black\_m\_boxes1.world -

 ![world1](https://github.com/Vladi-shar/ROS_Object_Manipulation/blob/master/Images/mov_boxes1.png)
 black\_m\_boxes2.world

 ![world2](https://github.com/Vladi-shar/ROS_Object_Manipulation/blob/master/Images/mov_boxes2.png)
 
Both located in the directory:







Web client

Install nodejs and npm express and run the following command:

Open a web browser and enter the following link:

[http://localhost:3000/main.html](http://localhost:3000/main.html)

Choose the desired board size

 ![site1](https://github.com/Vladi-shar/ROS_Object_Manipulation/blob/master/Images/site1.png)
 
After choosing the desired board size, you may insert obstacles and the target objects.

Note that the robotic arm is located to the right of the board.

The green squares are obstacles.

The red square is the goal.

Choosing between green and red squares is done by clicking the switch.

 ![site2](https://github.com/Vladi-shar/ROS_Object_Manipulation/blob/master/Images/site2.png)
Hitting the send button will return the instructions string if one exists.

 ![site3](https://github.com/Vladi-shar/ROS_Object_Manipulation/blob/master/Images/site3.png)
