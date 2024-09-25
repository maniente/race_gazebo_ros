# RaceGazeboRos
This project will be the Exercise 2 of the course Robotic Programming

## Requests
* Implement your own Gazebo simulation with sensors and other plugins for a simple mechatronic/robotic application.
* Bridge the topics from ignition_transport to ROS.
* Write a Python/C++ script to subscribe/publish to ROS topics and add functionality to your application.
* Create a ROS package and build your application as a ROS package, so that you can launch the package using ‘ros2 launch’ command as in Section 3.

## Idea
Race simulation: a race among robots in a strainght circuit like a 100m olimic race. Each robot have an own lane and at the and of each lane there is a sensor to detect the passage. On the end line there is also a camera to take a foto for a possible photo finish.

## Software Realization
a Ros node send on a topic a signal to start the race, this set also random velocity at each robot, start also a timer. When a robot trigger a sensor the sensor(Gazebo) send on topic the number of the robot , then the node on ros make a rank with the timer. 

## Possible implemantations
* launch of the world with a parametric number of robot
* show the photo finish picture if is under a gap \epsilon
* timer

# Road Map
* world creation
  * 
* Ros node to start the race
  * 
* Ros node to wait the end of the race
* create a launch file
