# ME495 Sensing, Navigation and Machine Learning For Robotics
* Ayush Gaggar
* Winter 2023
# Package List
This repository consists of several ROS packages
- <PACKAGE1> - <one sentence description>

# Nuturtle  Description
## TODO: Fill this in
URDF files for Nuturtle <Name Your Robot>
* `<Command Here>` to see the robot in rviz.
* `<Command Here>` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)
# Launch File Details
* `<Command To Show Arguments of load_one.launch.py>`
  `<Output of the Above Command>`
* `<Command To Show Arguments of load_all.launch.py>`
  `<Output of the Above Command>`

# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
        1) compute the L2 magnitude of the vector, and divide each element of the vector inside a function
        2) normalize vectors by default during construction, inside contructors
        3) use a c++ linear algebra library, like eigen

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
        1) pro: simple straight forward logic; con: more than one logical operator inside function, a little more costly at runtime per transaction
        2) pro: all vectors are normalized (useful if you will end up normalizing *every* vector anyways). cons: YAGNI - costly if vector normalization isn't used too often, as these operations will happen for every single vector that's initialized
        3) pro: simple implementation, cons: doing #include copies entire library into code, which takes far more memory; also, you're using code that you have not written personally, so there's a greater chance of acccidentally implementing the method incorrectly. 

   - Which of the methods would you implement and why?
        I would use the first method that I described, since it's the most logical (to me), and is optimal if vector normalization occurs, but only infrequently.

2. What is the difference between a class and a struct in C++?


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

Worked With <List anyone you worked with here or change to nobody if nobody>



 Enter transform T_{a,b}:
>deg: 90 x: 0 y: 1
 Enter transform T_{b,c}:
>deg: 90 x: 1 y: 0
 T_{a,b}: deg: 90 x: 0 y: 1
 T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
 T_{b,c}: deg: 90 x: 1 y: 0
 T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
 T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
 T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
 Enter vector v_b:
>1 1
 v_bhat: [0.707107 0.707107]
 v_a: [-1 2]
 v_b: [1 1]
 v_c: [1 1.11022e-16]
 Enter twist V_b:
>1 1 1
 V_a [1 0 1]
 V_b [1 1 1]
 V_c [1 2 -1]