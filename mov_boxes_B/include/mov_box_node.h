//
// Created by vladi on 5/17/19.
//



// ROS
#include <ros/ros.h>


// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++
#include <iostream>
#include <string>
#include <sstream>
//#include "../include/qnode.hpp"
#include "std_msgs/String.h"


#include <eigen3/Eigen/Eigen>
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include <manipulator_h_base_module_msgs/JointPose.h>
#include <manipulator_h_base_module_msgs/KinematicsPose.h>
#include <manipulator_h_base_module_msgs/GetJointPose.h>
#include <manipulator_h_base_module_msgs/GetKinematicsPose.h>
#include <regex>
#include <std_msgs/Float64.h>
//using namespace Qt;
//using namespace mov_box_node;

#ifndef SRC_MOV_BOX_NODE_H
#define SRC_MOV_BOX_NODE_H

#endif //SRC_MOV_BOX_NODE_H

class BoxMover {
public:

    enum                align_enum { left_align, right_align, top, center };
    enum                direction_enum { left, right, forward, backward, up, down };
    enum                finger_enum {left_upper, left_lower, right_upper, right_lower};

    BoxMover();
    virtual ~BoxMover();
    int init_pos();
    bool init(int argc_, char** argv_);
    void run();
    int run_instructions( std::string s);
    void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);
      int set_gripper_mov_object_pos();
          int mov_head_to_y_then_x(int x, int y, align_enum align);
    int mov_head_to_x_then_y(int x, int y, align_enum align);
    int mov_fingers(float left_uper, float left_lower, float right_upper, float right_lower);
      int open_gripper();
   int close_griper();
    int send_mov_message(float x, float y, float z);

private:

    int flag;

    align_enum          head_align = center;
    float               cell_length = 0.15;
    int                 head_row = 5;
    int                 head_col = 2;
    float               head_x = 0.191;
    float               head_y = 0;
    float               head_z = 0.19;
    ros::Publisher      chatter_publisher_;
    ros::Publisher      ini_pose_msg_pub_;
    ros::Publisher      set_mode_msg_pub_;
    ros::Publisher      left_lower_finger_msg_pub_;
    ros::Publisher      right_lower_finger_msg_pub_;
    ros::Publisher      left_upper_finger_msg_pub_;
    ros::Publisher      right_upper_finger_msg_pub_;
    ros::Publisher      kinematics_pose_msg_pub_;
    ros::ServiceClient  get_kinematics_pose_client_;
    ros::Subscriber     status_msg_sub_;
    //ros::NodeHandle     nodeHandler;
//    int                 init_argc_;
//    char**              init_argv_;


    int perform_insctruction(int x, int y, direction_enum dir, int next_x, int next_y, direction_enum dir2);




    float get_offset_y(align_enum align);

    float get_offset_x(align_enum align);

    int mov_forward(int units, align_enum align);

    int mov_backward(int units, align_enum align);

    int mov_left(int units, align_enum align);

    int mov_right(int units, align_enum align);


   
    
    int set_mode();
    void waitForEndTragectory();
   int mov_finger(std_msgs::Float64 finger_value, finger_enum finger);
 
 
   

};
