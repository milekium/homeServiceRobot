/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int16.h"
#include "add_markers/MarkerInfo.h"
#include <iostream>

class MarkersManager
{
public:
  MarkersManager()
  {
    // Init Service and Publishers
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
    orders_pub_ = n_.advertise<std_msgs::Int16>("add_markers/orders", 1); 
    marker_update_ = n_.advertiseService("/add_markers/orders_update", &MarkersManager::handle_markers_update, this);
      
    // Set Ros Rate
    ros::Rate r(1);
    //Init first Marker_id
    marker_id_.data = 1;

    while (ros::ok())
    { 
      ros::spinOnce();

      publish_marker_by_id();

      r.sleep();
    }
  } 
  bool publish_marker_by_id(){
    orders_pub_.publish(marker_id_);
  }
  bool publish_marker_to_map(int id){
    ROS_INFO_STREAM("Publish mark");
    // Init Marker
    marker_.header.frame_id = "map";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = "basic_shapes";    
    marker_.type = visualization_msgs::Marker::CUBE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = id;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.2;
    marker_.scale.y = 0.2;
    marker_.scale.z = 0.2;
    if(markers_db_[id][0] == 1)
    {
      marker_.pose.position.x = markers_db_[id][2];
      marker_.pose.position.y = markers_db_[id][3];
      marker_.color.r = 0.0f;
      marker_.color.g = 1.0f;
      marker_.color.b = 0.0f;
      marker_.color.a = 1.0;
    }
    else{
      marker_.pose.position.x = markers_db_[id][5];
      marker_.pose.position.y = markers_db_[id][6];
      marker_.color.r = 0.4f;
      marker_.color.g = 0.2f;
      marker_.color.b = 0.7f;
      marker_.color.a = 1.0;
    }
    marker_pub_.publish(marker_);
    return 1;
  } 
  bool unpublish_marker_to_map(int id){
    ROS_INFO_STREAM("UnPublish mark");
    marker_.action = visualization_msgs::Marker::DELETE;
    marker_.id = id;
    marker_pub_.publish(marker_);
    return 1;
  } 
  bool handle_markers_update(add_markers::MarkerInfo::Request& req, add_markers::MarkerInfo::Response& res)
  {
    ROS_INFO_STREAM("Update mark");
    switch(req.mark_status){
    case 0: // waiting (no robot handles)
      markers_db_[req.mark_id][0] = 1;
      res.mark_status = 1;
      res.mark_id = req.mark_id;
      res.linear_x = markers_db_[req.mark_id][2];
      res.linear_y = markers_db_[req.mark_id][3];
      res.angular_z = markers_db_[req.mark_id][4];
      marker_id_.data = req.mark_id + 1;
      publish_marker_to_map(req.mark_id);
      break;
    case 1: // to pickup (robot is on his way to pickup)
      markers_db_[req.mark_id][0] = 2;
      res.mark_status = 2;
      res.mark_id = req.mark_id;
      res.linear_x = markers_db_[req.mark_id][5];
      res.linear_y = markers_db_[req.mark_id][6];
      res.angular_z = markers_db_[req.mark_id][7];  
      ros::Duration(5.0).sleep();
      unpublish_marker_to_map(req.mark_id);
      ros::Duration(5.0).sleep();
      publish_marker_to_map(req.mark_id);            
    break;
    case 2: // pickup -> dropoff (robot is on his way to dropoff)
      markers_db_[req.mark_id][0] = 3;
      res.mark_status = 3;
      res.mark_id = req.mark_id;
      res.linear_x = -1.0;
      res.linear_y = -1.0;
      res.angular_z = -1.0;  
      ros::Duration(5.0).sleep();
      unpublish_marker_to_map(req.mark_id);    
      break;
    case 3: // complete.
      return true;      
      break;
    }

    // Return a response message
    ROS_INFO_STREAM(res);
    return true;
   };
private:
  ros::NodeHandle n_;
  ros::Publisher marker_pub_;
  ros::Publisher orders_pub_; 
  ros::ServiceServer marker_update_;
  visualization_msgs::Marker marker_;
  add_markers::MarkerInfo order_;
  std_msgs::Int16 marker_id_;
  FILE* markers_file_;
  std::vector< std::vector<double> > markers_db_ {{0, 0, 0, 0, 0, 0, 0, 0},
                                                    {0, 1, -1, 1.5, 3.14, 3.6, 2, 1.57},
                                                    {0, 2, 1, 0, -1.57, -1, 1, -3.14},
                                                    {0, 3, 3.6, 2, 3.14, -1.0, 1.8, 0.5},
                                                    {0, 4, 1, 0, -1.57, -1, 1, -3.14},
                                                    {0, 5, 3.6, 2, 3.14, -1.0, 1.8, 0.5},
                                                    {0, 6, 1, -0.50, -1.57, -1, 2, -3.14},
                                                    {0, 7, 3.6, 2, 3.14, 3.6, 3, 0.5}};                                                    ;
};

int main( int argc, char** argv )
{
  //Initiate Ros
  
  ros::init(argc, argv, "add_markers");

  //Create an object of class MarkersManager that will take care of everything
  MarkersManager SAPObject;

  // Handle ROS communication events
  ros::spin();

  return 0;
 
}
