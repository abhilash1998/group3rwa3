#ifndef __ARM_H__
#define __ARM_H__

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>
// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/LogicalCameraImage.h>
// custom
#include "utils.h"
#include <nist_gear/LogicalCameraImage.h>

namespace motioncontrol {


    /**
     * @brief class for the Gantry robot
     *
     */
    class Arm {
        public:
        /**
        * @brief Struct for preset locations
        * @todo Add new preset locations here if needed
        */
        typedef struct ArmPresetLocation {
            std::vector<double> arm_preset;  //9 joints
            std::string name;
        } start, bin, agv, grasp;

        Arm(ros::NodeHandle& node_handle);
        /**
         * @brief Initialize the object
         */
        void init();

        /**
         * @brief Returns true if the part is sucessfully picked up by the robotic arm.
         * 
         * @param part_type - Name of the part which needs to be picked up
         * @param part_pose - Pose of the part from which it need to be picked up
         * @param ss - integer to switch values of between bin and tray.
         after which it slows down for attaching the object.
         * @return true - True if sucessfully picked
         * @return false - False if not picked up 
         */
        bool pickPart(std::string part_type, geometry_msgs::Pose part_pose, int ss);
        
        /**
         * @brief Places the part to the given desired location on the given agv nad return true when the part is sucessfully kept. 
         * 
         * @param part_init_pose - Global pose of the part in the bin or current position at the starting of this function
         * @param part_goal_pose - The pose at which the part needs to be kept on the agv with reference to the camera frame.
         * @param agv - AGV name on which the part needs to be kept
         * @return true - Part sucessfully picked up
         * @return false - Part was not picked up sucessfully
         */
        bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv);
        void testPreset(const std::vector<ArmPresetLocation>& preset_list);
        void movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv);
        void activateGripper();
        void deactivateGripper();

        /**
         * @brief check the part of the pose after the part is dettached from the robot.
         If the orientation and position is right according to the order id then this function calls 
         pickpart and place part to correctly place the  object with respect to the order.
         * 
         * @param target_pose_in_world - position where the object needs to be kept on agv with respect to the camera frame
         * @param agv - Agv where the part is to be kept
         */
        void check_part_pose(geometry_msgs::Pose target_pose_in_world,std::string agv);

        /**
         * @brief Callback for the subscriber of logical camera 1
         * 
         * @param msg data from the subscriber of logical camera
         */
        void qualityControl1Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);


        void rotate_the_part(std::array<double,3>  part_orientation,geometry_msgs::Pose   target_pose,tf2::Quaternion q_current,std::string &agv,std::string &angle);
        
        /**
         * @brief Callback for the subscriber of logical camera 2
         * 
         * @param msg data from the subscriber of logical camera
         */
        void qualityControl2Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        
        /**
         * @brief Callback for the subscriber of logical camera 3
         * 
         * @param msg data from the subscriber of logical camera
         */
        void qualityControl3Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
        
        /**
         * @brief Callback for the subscriber of logical camera 4
         * 
         * @param msg data from the subscriber of logical camera
         */
        void qualityControl4Callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
          * @brief Get the quality of objects in quality camera1 data 
          * 
          * @return true - True if faulty part is found
          * @return false - False if not found
          */       
        bool& get_quality_camera1_data(){
            return quality_camera_1;
        }

        /**
          * @brief Get the quality of objects in quality camera2 data 
          * 
          * @return true - True if faulty part is found
          * @return false - False if not found
          */
        bool& get_quality_camera2_data(){
            return quality_camera_2;
        }

        /**
          * @brief Get the quality of objects in quality camera3 data 
          * 
          * @return true - True if faulty part is found
          * @return false - False if not found
          */
        bool& get_quality_camera3_data(){
            return quality_camera_3;
        }

        /**
          * @brief Get the quality of objects in quality camera4 data 
          * 
          * @return true - True if faulty part is found
          * @return false - False if not found
          */
        bool& get_quality_camera4_data(){
            return quality_camera_4;
        }

        /**
         * @brief Move the joint linear_arm_actuator_joint only
         *
         * The joint position for this joint corresponds to the y value
         * in the world frame. For instance, a value of 0 for this joint
         * moves the base of the robot to y = 0.
         *
         * @param location A preset location
         */
        void moveBaseTo(double linear_arm_actuator_joint_position);
        nist_gear::VacuumGripperState getGripperState();

        

        // Send command message to robot controller
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        void goToPresetLocation(std::string location_name);

        //--preset locations;
        start home1_, home2_;
        agv agv1_, agv2_, agv3_, agv4_;

        //-- pointer variable for the current part number
        int* counter;

        //-- boolean variables for faulty parts detected
        bool quality_camera_1;
        bool quality_camera_2;
        bool quality_camera_3;
        bool quality_camera_4;
        bool quality_camera[4];

        /**
         * @brief Get the camera frame of moving object/object that is being placed 
         * 
         * @return std::string - camera name which detected the object
         */
        std::string get_camera_frame_of_moving_object(){
            return camera_frame_of_moving_object;
        } 

        //-- string variable contatining camera frame name
        std::string camera_frame_of_moving_object;

        private:
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options arm_options_;
        moveit::planning_interface::MoveGroupInterface arm_group_;
        sensor_msgs::JointState current_joint_states_;
        control_msgs::JointTrajectoryControllerState arm_controller_state_;

        nist_gear::VacuumGripperState gripper_state_;
        // gripper state subscriber
        ros::Subscriber gripper_state_subscriber_;
        // service client
        ros::ServiceClient gripper_control_client_;
        // publishers
        ros::Publisher arm_joint_trajectory_publisher_;
        // joint states subscribers
        ros::Subscriber arm_joint_states_subscriber_;
        // controller state subscribers
        ros::Subscriber arm_controller_state_subscriber_;

        std::string part_type_name;

        /**
         * @brief Gets the pointer of counter or the number of the current part 
         * 
         * @return int* 
         */
        int *get_counter(){
            return counter;
        }

        /**
         * @brief Get the part type name 
         * 
         * @return auto - string of part type name
         */
        auto get_part_type_name(){
            return part_type_name;
        }

        // callbacks
        void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
        void arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

        void check_faulty_part(std::string part_type,geometry_msgs::Pose part_pose_in_frame,std::string agv);
        
        ros::Subscriber Arm_quality_control_sensor_1_subscriber;
        /*!< subscriber to the topic /ariac/quality_control_sensor_2 */
        ros::Subscriber Arm_quality_control_sensor_2_subscriber;
        ros::Subscriber Arm_quality_control_sensor_3_subscriber;
        ros::Subscriber Arm_quality_control_sensor_4_subscriber;
    
    };
}//namespace
#endif
