# HOME SERVICE ROBOT

Home service Robot is a robot simulation able to perform autonomous task as mapping the enviropment, localization, navigation, obstacle avoidance..ect. for the main objective for the robot is to perform a pickup - dropoff task inside a enviropment using ROS.

# Robot mission:
Green box (Rviz Mark) will apear inside the building, indicating a pickup area. The robot mission is to travel to the pickup area and simulate a pickup operation of 5 sec. after this time, a blue box appears indication the dropoff area, then robot will travel to the dropoff area to simulate dropoff operation.

In order to complete the simulation:
* The robot is deployed in a simulated world.
* The robot uses sensors (Laser scaner) to gather information about its sorroundings.
* The robot can use the gathered exploration information to generate a map of the world.
* The robot can navigate inside the world planning its path using the generated map.

Timeline:
* The robot recives a pickup mark coordanates, coordanates in the world.
* The robot autonomously travels to the mark using the generated map, localization and navigations modules.
* The robot arrives to the pickup mark and waits for delivery orders.
* The robot recieves a delivery mark coordinates and plans its navigations to it.
* The robot arrives to the delivery mark and gets a : mission acomplished!!.

Robot Operation Packages:
The robot pickup and delivery process is accomplished by add_markers and pick_objects packages; the firsts sets the marks for the robot to travel to, and the second commands the robot to travel to this marks.

# Custom Packages and configs:

## Add Markers package:
Everytime an order is ready for delivery, the package publishes a message with the mark id for the robot to pickup, process as:
* The marker should initially be published at the pickup zone. 
* 5 seconds after the robot arrives to the pickup marker, mark is hidden. 
* Then after another 5 seconds it should appear a mark in the drop off zone.
* After robot arrives to the drop off mark, the mark should be hidden.
* Robot should then go back to the wait zone while subscribing to get new mark id.
* Proccess continues with a new pickup mark.

## Pick Objects package:
Commands the navigation and path planning for the robot by generating the actual goal for the robot movement.
* The robot stay in a waiting state/ waiting area, while subscribed to marks channel; waits for a new order to come.
* The robot sees a new mark id Msg and sends request to the service markupdate to get this order info.
* The robot gets order information if order is adjudicated to the robot.
* The robot moves to the pickup area, shown with a green pickup mark.
* Ones the robot reach the pickup mark, the robot sends msg info to the service update marks, the response is the dropoff coordinates.
* Ones the robot reach the dropoff mark, the robot sends another msg to the service update to indicate is arrival.
* Ones the dropoff is complete, the robot returns to waiting state and lisen for new orders to come.

### Services:     
/add_markers/orders_update
*    Server (add_markers.cpp): keep track of the delivery orders states by updating the marks.
*    Client (pick_objects.cpp): Use this service to retrieve coordinates each time it accomplished a goal.
    
### Topics:
add_markers/orders:
*    Publisher (add_markers.cpp): Publish Markers id, simulating Orders, ready for pickup-dropoff.
*    Subscriber(pick_objects.cpp) When robot is waiting or last pickup-dropoff misssion is finish, robot subscribe to get a new delivery Orders. When order/marker id is received, robot unsubscribe from channel. this is intended in case, more than one robot is used. 

## Simulation Package folders:
*  map: Inside this directory, you will store your gazebo world file and the map generated from SLAM.
*  scripts: Inside this directory, you’ll store your shell scripts.
*  rvizConfig: Inside this directory, you’ll store your customized rviz configuration files.
*  pick_objects: You will write a node that commands your robot to drive to the pickup and drop off zones.
*  add_markers: You will write a node that model the object with a marker in rviz.

# Official ROS packages used:
* gmapping: With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
* turtlebot_teleop: With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
* turtlebot_rviz_launchers: With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
* turtlebot_gazebo: With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.
*  Move Robot Package: Robot uses move_base package to set destination coordinates.

## Localization, Mapping, Navigation, Navigation Goals
Robot navigation goals are set in the script, where multiple goals can be set. Also, turtlebot_teleop package is present, so the robot can be operated manually with the keyboard or similar.

# Install and Run:
Run the following code to download, install and run the code
```
git clone https://github.com/milekium/homeServiceRobot.git
cd homeServiceRobot
source env.sh
catking_make
./src/scripts/home_service.sh
```

# Partial Runs and tests
After building the code, you can run partials of the code, or test by runing the scripts inside the folder scr/scripts.
