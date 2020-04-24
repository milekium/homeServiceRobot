#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  // set up mission planning
  bool mission_acomplished = false;
  bool second_tarject = false;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  while(!mission_acomplished){
    if(!second_tarject && !mission_acomplished){
      // Define a position and orientation for the robot to reach
      goal.target_pose.pose.position.x = -1.0;
      goal.target_pose.pose.position.y = 1.50;
      goal.target_pose.pose.orientation.w = 1.57;

       // Send the goal position and orientation for the robot to reach
      ROS_INFO("Send Pickup Position goal.");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();
      second_tarject = true;
    } 
    // Check if the robot reached its goal
    else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && second_tarject){
      ROS_INFO("Pick up Position reached.");
      // Wait 5 sec for move_base action server to come up
      ROS_INFO("Enjoying frist position for 5 secs.");
      ros::Duration(5.0).sleep();
      // Define a second position and orientation for the robot to reach
      goal.target_pose.pose.position.x = 3.6;
      goal.target_pose.pose.position.y = 2.0;
      goal.target_pose.pose.orientation.w = 0.50;
      // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending Second goal");
      ac.sendGoal(goal);
      // Wait an infinite time for the results
      ac.waitForResult();
      mission_acomplished = true;      
    }
    else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && mission_acomplished){
      ROS_INFO("Hooray, the base moved to second  position, !mission accomplished!");
    }  
    else{
      ROS_INFO("The base failed to move for some reason");
      mission_acomplished = true;
    }
   
  }
  return 0;
}
