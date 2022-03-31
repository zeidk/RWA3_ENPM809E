#include "arm.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>
#include "utils.h"
#include <math.h>

namespace motioncontrol {
    /////////////////////////////////////////////////////
    Arm::Arm(ros::NodeHandle& node) : node_("/ariac/kitting"),
        planning_group_("/ariac/kitting/robot_description"),
        arm_options_("kitting_arm", planning_group_, node_),
        arm_group_(arm_options_)
    {
        ROS_INFO_STREAM("[Arm] constructor called... ");

    }


    /////////////////////////////////////////////////////
    void Arm::init()
    {
        // make sure the planning group operates in the world frame
        // check the name of the end effector
        // ROS_INFO_NAMED("init", "End effector link: %s", arm_group_.getEndEffectorLink().c_str());


        // publishers to directly control the joints without moveit
        arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/kitting/kitting_arm_controller/command", 10);
        // joint state subscribers
        arm_joint_states_subscriber_ =
            node_.subscribe("/ariac/kitting/joint_states", 10, &Arm::arm_joint_states_callback_, this);
        // controller state subscribers
        arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/kitting/kitting_arm_controller/state", 10, &Arm::arm_controller_state_callback, this);
        // gripper state subscriber
        gripper_state_subscriber_ = node_.subscribe(
            "/ariac/kitting/arm/gripper/state", 10, &Arm::gripper_state_callback, this);
        // controller state subscribers
        gripper_control_client_ =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
        gripper_control_client_.waitForExistence();


        // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the arm are in this order:
        // - linear_arm_actuator_joint
        // - shoulder_pan_joint
        // - shoulder_lift_joint
        // - elbow_joint
        // - wrist_1_joint
        // - wrist_2_joint
        // - wrist_3_joint

        double linear_arm_actuator_joint{ 0 };
        double shoulder_pan_joint{ 0 };
        double shoulder_lift_joint{ -1.25 };
        double elbow_joint{ 1.74 };
        double wrist_1_joint{ -2.06 };
        double wrist_2_joint{ -1.51 };
        double wrist_3_joint{ 0 };

        //home position
        home1_.arm_preset = { 0, 0, -1.25, 1.74, -2.04, -1.57, 0 };
        home1_.name = "home1";
        home2_.arm_preset = { 0, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        home2_.name = "home2";
        agv1_.arm_preset = { 3.83, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv1_.name = "agv1";
        agv2_.arm_preset = { 0.83, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv2_.name = "agv2";
        agv3_.arm_preset = { -1.83, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv3_.name = "agv3";
        agv4_.arm_preset = { -4.33, -M_PI, -1.25, 1.74, -2.04, -1.57, 0 };
        agv4_.name = "agv4";


        // raw pointers are frequently used to refer to the planning group for improved performance.
        // to start, we will create a pointer that references the current robot’s state.
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("kitting_arm");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();
        // next get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);
    }

    //////////////////////////////////////////////////////
    void Arm::moveBaseTo(double linear_arm_actuator_joint_position) {
        // get the current joint positions
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("kitting_arm");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();

        // get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

        // next, assign a value to only the linear_arm_actuator_joint
        joint_group_positions_.at(0) = linear_arm_actuator_joint_position;

        // move the arm
        arm_group_.setJointValueTarget(joint_group_positions_);
        arm_group_.move();
    }
    //////////////////////////////////////////////////////
    void Arm::movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv) {
        //convert goal_in_tray_frame into world frame
        auto init_pose_in_world = motioncontrol::transformToWorldFrame(camera_frame);
        // ROS_INFO_STREAM(init_pose_in_world.position.x << " " << init_pose_in_world.position.y);
        auto target_pose_in_world = motioncontrol::transformToWorldFrame(goal_in_tray_frame, agv);
        if (pickPart(part_type, init_pose_in_world)) {
            placePart(init_pose_in_world, goal_in_tray_frame, agv);
        }
    }
    /////////////////////////////////////////////////////
    nist_gear::VacuumGripperState Arm::getGripperState()
    {
        return gripper_state_;
    }

    /**
     * @brief Pick up a part from a bin
     *
     * @param part Part to pick up
     * @return true Part was picked up
     * @return false Part was not picked up
     *
     * We use the group full_gantry_group_ to allow the robot more flexibility
     */
    bool Arm::pickPart(std::string part_type, geometry_msgs::Pose part_init_pose) {
        arm_group_.setMaxVelocityScalingFactor(1.0);

        
        
        moveBaseTo(part_init_pose.position.y);

        // // move the arm above the part to grasp
        // // gripper stays at the current z
        // // only modify its x and y based on the part to grasp
        // // In this case we do not need to use preset locations
        // // everything is done dynamically
        // arm_ee_link_pose.position.x = part_init_pose.position.x;
        // arm_ee_link_pose.position.y = part_init_pose.position.y;
        // arm_ee_link_pose.position.z = arm_ee_link_pose.position.z;
        // // move the arm
        // arm_group_.setPoseTarget(arm_ee_link_pose);
        // arm_group_.move();

        // Make sure the wrist is facing down
        // otherwise it will have a hard time attaching a part
        geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        arm_ee_link_pose.orientation.x = flat_orientation.getX();
        arm_ee_link_pose.orientation.y = flat_orientation.getY();
        arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        arm_ee_link_pose.orientation.w = flat_orientation.getW();
        
        // post-grasp pose 3
        // store the pose of the arm before it goes down to pick the part
        // we will bring the arm back to this pose after picking up the part
        auto postgrasp_pose3 = part_init_pose;
        postgrasp_pose3.orientation = arm_ee_link_pose.orientation;
        postgrasp_pose3.position.z = arm_ee_link_pose.position.z;

        // preset z depending on the part type
        // some parts are bigger than others
        // TODO: Add new z_pos values for the regulator and the battery
        double z_pos{};
        if (part_type.find("pump") != std::string::npos) {
            z_pos = 0.859;
        }
        if (part_type.find("sensor") != std::string::npos) {
            z_pos = 0.81;
        }

        // flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        // arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        // arm_ee_link_pose.orientation.x = flat_orientation.getX();
        // arm_ee_link_pose.orientation.y = flat_orientation.getY();
        // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        // arm_ee_link_pose.orientation.w = flat_orientation.getW();

        
        // set of waypoints the arm will go through
        std::vector<geometry_msgs::Pose> waypoints;
        // pre-grasp pose: somewhere above the part
        auto pregrasp_pose = part_init_pose;
        pregrasp_pose.orientation = arm_ee_link_pose.orientation;
        pregrasp_pose.position.z = z_pos + 0.06;

        // grasp pose: right above the part
        auto grasp_pose = part_init_pose;
        grasp_pose.orientation = arm_ee_link_pose.orientation;
        grasp_pose.position.z = z_pos + 0.03;

        waypoints.push_back(pregrasp_pose);
        waypoints.push_back(grasp_pose);

        // activate gripper
        // sometimes it does not activate right away
        // so we are doing this in a loop
        while (!gripper_state_.enabled) {
            activateGripper();
        }

        // move the arm to the pregrasp pose
        arm_group_.setPoseTarget(pregrasp_pose);
        arm_group_.move();

        
        /* Cartesian motions are frequently needed to be slower for actions such as approach
        and retreat grasp motions. Here we demonstrate how to reduce the speed and the acceleration
        of the robot arm via a scaling factor of the maxiumum speed of each joint.
        */
        arm_group_.setMaxVelocityScalingFactor(0.05);
        arm_group_.setMaxAccelerationScalingFactor(0.05);
        // plan the cartesian motion and execute it
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm_group_.execute(plan);

        ros::Duration(sleep(2.0));

        // move the arm 1 mm down until the part is attached
        while (!gripper_state_.attached) {
            grasp_pose.position.z -= 0.001;
            arm_group_.setPoseTarget(grasp_pose);
            arm_group_.move();
            ros::Duration(sleep(0.5));
        }
        
            arm_group_.setMaxVelocityScalingFactor(1.0);
            arm_group_.setMaxAccelerationScalingFactor(1.0);
            ROS_INFO_STREAM("[Gripper] = object attached");
            ros::Duration(sleep(2.0));
            arm_group_.setPoseTarget(postgrasp_pose3);
            arm_group_.move();

            return true;
        
    }


    /////////////////////////////////////////////////////
    bool Arm::placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_pose_in_frame, std::string agv)
    {
        goToPresetLocation(agv);
        // get the target pose of the part in the world frame
        auto target_pose_in_world = motioncontrol::transformToWorldFrame(
            part_pose_in_frame,
            agv);


        geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
        arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        arm_ee_link_pose.orientation.x = flat_orientation.getX();
        arm_ee_link_pose.orientation.y = flat_orientation.getY();
        arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        arm_ee_link_pose.orientation.w = flat_orientation.getW();

        // store the current orientation of the end effector now
        // so we can reuse it later
        tf2::Quaternion q_current(
            arm_ee_link_pose.orientation.x,
            arm_ee_link_pose.orientation.y,
            arm_ee_link_pose.orientation.z,
            arm_ee_link_pose.orientation.w);
        
        // move the arm above the agv
        // gripper stays at the current z
        // only modify its x and y based on the part to grasp
        // In this case we do not need to use preset locations
        // everything is done dynamically
        arm_ee_link_pose.position.x = target_pose_in_world.position.x;
        arm_ee_link_pose.position.y = target_pose_in_world.position.y;
        // move the arm
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPoseTarget(arm_ee_link_pose);
        arm_group_.move();

        


        // orientation of the part in the bin, in world frame
        tf2::Quaternion q_init_part(
            part_init_pose.orientation.x,
            part_init_pose.orientation.y,
            part_init_pose.orientation.z,
            part_init_pose.orientation.w);
        // orientation of the part in the tray, in world frame
        tf2::Quaternion q_target_part(
            target_pose_in_world.orientation.x,
            target_pose_in_world.orientation.y,
            target_pose_in_world.orientation.z,
            target_pose_in_world.orientation.w);

        // relative rotation between init and target
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();

        // orientation of the gripper when placing the part in the tray
        target_pose_in_world.orientation.x = q_rslt.x();
        target_pose_in_world.orientation.y = q_rslt.y();
        target_pose_in_world.orientation.z = q_rslt.z();
        target_pose_in_world.orientation.w = q_rslt.w();
        target_pose_in_world.position.z += 0.2;

        arm_group_.setMaxVelocityScalingFactor(0.1);
        arm_group_.setPoseTarget(target_pose_in_world);
        arm_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();

        arm_group_.setMaxVelocityScalingFactor(1.0);
        goToPresetLocation("home2");

        return true;
        // TODO: check the part was actually placed in the correct pose in the agv
        // and that it is not faulty
    }
    /////////////////////////////////////////////////////
    void Arm::gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
    {
        gripper_state_ = *gripper_state_msg;
    }
    /////////////////////////////////////////////////////
    void Arm::activateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = true;
        gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Arm][activateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Arm::deactivateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = false;
        gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Arm][deactivateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Arm::goToPresetLocation(std::string location_name)
    {

        ArmPresetLocation location;
        if (location_name.compare("home1") == 0) {
            location = home1_;
        }
        else if (location_name.compare("home2") == 0) {
            location = home2_;
        }
        else if (location_name.compare("agv1") == 0) {
            location = agv1_;
        }
        else if (location_name.compare("agv2") == 0) {
            location = agv2_;
        }
        else if (location_name.compare("agv3") == 0) {
            location = agv3_;
        }
        else if (location_name.compare("agv4") == 0) {
            location = agv4_;
        }
        joint_group_positions_.at(0) = location.arm_preset.at(0);
        joint_group_positions_.at(1) = location.arm_preset.at(1);
        joint_group_positions_.at(2) = location.arm_preset.at(2);
        joint_group_positions_.at(3) = location.arm_preset.at(3);
        joint_group_positions_.at(4) = location.arm_preset.at(4);
        joint_group_positions_.at(5) = location.arm_preset.at(5);
        joint_group_positions_.at(6) = location.arm_preset.at(6);

        arm_group_.setJointValueTarget(joint_group_positions_);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // check a plan is found first then execute the action
        bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            arm_group_.move();
    }


    ///////////////////////////
    ////// Callback Functions
    ///////////////////////////

    /////////////////////////////////////////////////////
    void Arm::arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
    {
        if (joint_state_msg->position.size() == 0) {
            ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
        }
        current_joint_states_ = *joint_state_msg;
    }

    /////////////////////////////////////////////////////
    void Arm::arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        arm_controller_state_ = *msg;
    }
}//namespace

