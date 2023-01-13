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
     There is no difference between a class and a struct *except* structs have all member variables public by default.


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
     The primary reason is due to C++ core guideline C.2, which says that since Transform2D has variables that should be private (and thus its class variables should not vary independently), Transform 2D should be a class; on the other hand, since Vector2D can independently vary its x and y, it should be a struct.
     Additionally, the design choice here is to make the member variables of class Transform2D private, so only its own objects can access those variables. As per guideline C.8, a class must be used rather than a struct any time a member is non-public.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
     As per guideline C.46, single argument constructors should, by default, use the "explicit" keyword to avoid any unintended conversions.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
   By default, member functions in a class should be declared as const. However, since the overloaded Transform2D::operator*=() is modifying the "this" object of the class, the function cannot be declared constant. As per guidleine Con.2, "a member function should be marked const unless it changes the object’s observable state"; the *= operator is modifying the Transform2D object itself, and so should not be defined as a const function.

Worked With <List anyone you worked with here or change to nobody if nobody>