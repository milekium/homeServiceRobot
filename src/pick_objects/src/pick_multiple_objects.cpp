#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int16.h"
#include "add_markers/MarkerInfo.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

class ObjectsManager
{
public:
  ObjectsManager()
  {
    ROS_INFO_STREAM("Init ObjectsManager");
    MoveBaseClient ac("move_base", true);   
  	client_ = n_.serviceClient<add_markers::MarkerInfo>("/add_markers/orders_update");
    
    // set up the frame parameters
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();

    // Set Ros Rate
    ros::Rate r(1);
    // Init markers_goal_
    markers_goal_ = {-1, 0, 0, 0, 0, 0};
    while (ros::ok())
    {
      ROS_INFO_STREAM("ros ok!");      
      if(markers_goal_[5] == 0)
      { //subscribe to get marker id to work on
        markers_goal_[5] = 2; 
        ROS_INFO_STREAM("Subscribed to add_markers/orders");
        sub_ = n_.subscribe("add_markers/orders", 1, &ObjectsManager::get_published_marker_id, this);  
      }
      while(markers_goal_[5] == 1)
      { 
        ROS_INFO("Sending goal: x:%f - y:%f -w:%f", (float)goal_.target_pose.pose.position.x,(float)goal_.target_pose.pose.position.y,(float)goal_.target_pose.pose.orientation.w);
        ac.sendGoal(goal_);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Hooray, status - %i ", (int)markers_goal_[0]);
          switch ((int)markers_goal_[0]){ // status
          case 0:
            break; 
          case 1: // arrived to pickup mark
            get_coordinates_and_go(markers_goal_[1], 1);   //go dropoff     
            break; 
          case 2: // arrived to dropoff mark
            get_coordinates_and_go(markers_goal_[1], 2);  //go home
            markers_goal_ = {-1, 0, 0, 0, 0, 0};
            break; 
          case 3:
            //reset markers_goal
            markers_goal_ = {-1, 0, 0, 0, 0, 0};
            break; 
          }          
          //markers_goal_[5] = 0;
        }
        else
        {
          ROS_INFO("The base failed to move for some reason");
        }
      r.sleep();
      }        
    ros::spinOnce();
    r.sleep(); 
    }
  }
  void get_published_marker_id(const std_msgs::Int16 marker_id)
  {
    get_coordinates_and_go(marker_id.data, 0);
  }
  void get_coordinates_and_go(int id, int status)
  {
    ROS_INFO_STREAM("get_coordinates_and_go");
    marker_.request.mark_status = status;
    marker_.request.mark_id = id;
    if (client_.call(marker_))
    {
      markers_goal_[0] = marker_.response.mark_status;
      markers_goal_[1] = marker_.response.mark_id;
      markers_goal_[2] = marker_.response.linear_x;
      markers_goal_[3] = marker_.response.linear_y;
      markers_goal_[4] = marker_.response.angular_z;
      markers_goal_[5] = 1;
      go_to_marker_goal();
      sub_.shutdown();
    }
    else
    {
      ROS_ERROR("Failed to call service");      
    }    
  }
  bool go_to_marker_goal()
  {
    ROS_INFO_STREAM("go_to_marker_goal");
    ROS_INFO("Marker Goal: %i %i %i ", (int)markers_goal_[2],(int)markers_goal_[3],(int)markers_goal_[4]);
    goal_.target_pose.pose.position.x = markers_goal_[2];
    goal_.target_pose.pose.orientation.w = markers_goal_[4];
    goal_.target_pose.pose.position.y = markers_goal_[3];    
    goal_.target_pose.pose.position.z = 0.0;    
    return true;
  }
private:
  ros::NodeHandle n_;
  ros::ServiceClient client_;
  ros::Subscriber sub_;  
  move_base_msgs::MoveBaseGoal goal_;
  add_markers::MarkerInfo marker_;
  std::vector<double> markers_goal_ ;
};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //Create an object of class PickObjects that will take care of everything
  ObjectsManager SAPObject;

  // Handle ROS communication events
  ros::spin();

  return 0;
}
