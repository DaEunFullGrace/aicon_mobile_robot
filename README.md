# Mobile Robot 
    This package is based on ROS(Robot Operating System)
    
  ## src/teleop.py
    It is based on turtlebot3_teleop package of ROS. 
    It can save destination of navigation into text file and map file at saved location using map_server.
    
    First, run turtlebot3_bringup or gazebo
    Second, run this teleop.py
    Third, you can move your robot using keyboard and save data.
  
  ## src/pose_intialization.py
    After map is saved, run turtlebot3_navigation.
    And then you can use this file to initialize robot's position in rviz
  
  ## src/navigation.py
    To move toward saved location, execute this file.
    It can load destination and publish goal position.
