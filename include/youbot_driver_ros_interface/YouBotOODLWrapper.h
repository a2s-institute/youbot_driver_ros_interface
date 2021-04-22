/******************************************************************************
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Sebastian Blumenthal
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef YOUBOTOODLWRAPPER_H_
#define YOUBOTOODLWRAPPER_H_

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "brics_actuator/msg/joint_positions.hpp"
#include "brics_actuator/msg/joint_velocities.hpp"

/* OODL includes */
#include "YouBotConfiguration.h"
#include <youbot_driver/youbot/JointTrajectoryController.hpp>
#include <youbot_driver/youbot/DataTrace.hpp>

//#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <actionlib/server/simple_action_server.h>

//typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

namespace youBot
{

/**
 * @brief Wrapper class to map ROS messages to OODL method calls for the youBot platform.
 */
class YouBotOODLWrapper
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    /**
     * @brief Constructor with a ROS node
     * @param node ROS node
     */
    YouBotOODLWrapper(rclcpp::Node::SharedPtr node);

    /**
     * @brief DEfault constructor.
     */
    virtual ~YouBotOODLWrapper();


    /* Coordination: */

    /**
     * @brief Initializes a youBot base.
     * @param baseName Name of the base. Used to open the configuration file e.g. youbot-base.cfg
     */
    void initializeBase(std::string baseName);

    /**
     * @brief Initializes a youBot base.
     * @param armName Name of the base. Used to open the configuration file e.g. youbot-manipulator.cfg
     * @param enableStandardGripper If set to true, then the default gripper of the youBot will be initialized.
     */
    void initializeArm(std::string armName);

    /**
     * @brief Stops all initialized elements.
     * Stops arm and/or base (if initialized).
     */
    void stop();


    /* Communication: */

    /**
     * @brief Callback that is executed when a commend for the base comes in.
     * @param youbotBaseCommand Message that contains the desired translational and rotational velocity for the base.
     */
    void baseCommandCallback(geometry_msgs::msg::Twist::SharedPtr youbotBaseCommand);

    /**
     * @deprecated
     * @brief Callback that is executed when a commend for the arm comes in.
     * @param youbotArmCommand Message that contains the desired joint configuration.
     *
     * Currently only the first configuration (JointTrajectoryPoint) per message is processed.
     * Velocity and acceleration values are ignored.
     */
    void armCommandCallback(trajectory_msgs::msg::JointTrajectory youbotArmCommand);

    /**
     * @brief Callback that is executed when a position command for the arm comes in.
     * @param youbotArmCommand Message that contains the desired joint configuration.
     * @param armIndex Index that identifies the arm
     */
    void armPositionsCommandCallback(brics_actuator::msg::JointPositions::SharedPtr youbotArmCommand, int armIndex);

    /**
     * @brief Callback that is executed when a velocity command for the arm comes in.
     * @param youbotArmCommand Message that contains the desired joint configuration.
     * @param armIndex Index that identifies the arm
     */
    void armVelocitiesCommandCallback(brics_actuator::msg::JointVelocities::SharedPtr youbotArmCommand, int armIndex);

    /**
     * @brief Callback that is executed when an action goal to perform a joint trajectory with the arm comes in.
     * @param youbotArmGoal Actionlib goal that contains the trajectory.
     * @param armIndex Index that identifies the arm
     */
    rclcpp_action::GoalResponse armJointTrajectoryGoalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> youbotArmGoal, unsigned int armIndex);

    /**
     * @brief Callback that is executed when an action goal of a joint trajectory is canceled.
     * @param youbotArmGoal Actionlib goal that contains the trajectory.
     * @param armIndex Index that identifies the arm
     */
    rclcpp_action::CancelResponse armJointTrajectoryCancelCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> youbotArmGoal, unsigned int armIndex);
    /**
     * @brief Callback that is executed when a goal to perform a joint trajectory with the arm comes in.
     * @param youbotArmGoal goal handle that contains the trajectory
     * @param armIndex Index that identifies the arm
     */
    void armJointTrajectoryAcceptCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> youbotArmGoal, unsigned int armIndex);
    
    void armJointTrajectoryExecute(const std::shared_ptr<GoalHandleFollowJointTrajectory> youbotArmGoal, unsigned int armIndex);
    /**
     * @brief Callback that is executed when a position command for the gripper comes in.
     * @param youbotGripperCommand Message that contains the desired joint configuration.
     * @param armIndex Index that identifies the arm
     */
    void gripperPositionsCommandCallback(brics_actuator::msg::JointPositions::SharedPtr youbotGripperCommand, int armIndex);

    /**
     * @brief Publishes all sensor measurements. Both for base and arm.
     *
     * Depending on what has been initialized before, either odometry and/or joint state valiues are published.
     * computeOODLSensorReadings needs to be executed before.
     */
    void publishOODLSensorReadings();
    
    /**
    * @brief Publishes status of base and arm as diagnostic and dashboard messages continuously
    */
    void publishArmAndBaseDiagnostics(double publish_rate_in_secs);

    /* Computation: */

    /**
     * @brief Mapps OODL values to ROS messages
     */
    void computeOODLSensorReadings();

    void switchOffBaseMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void switchOnBaseMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void switchOffArmMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response,
        int armIndex);

    void switchOnArmMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response,
        int armIndex);

    void calibrateArmCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response,
        int armIndex);

    void reconnectCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    /* Configuration: */

    /// Handle the aggregates all parts of a youBot system
    YouBotConfiguration youBotConfiguration;

private:

    YouBotOODLWrapper(); //forbid default constructor

    rclcpp::Node::SharedPtr node;

    /// Degrees of freedom for the youBot manipulator
    static const int youBotArmDoF = 5;

    /// Number of finger mounted on the gripper.
    static const int youBotNumberOfFingers = 2;

    /// Number of wheels attached to the base.
    static const int youBotNumberOfWheels = 4;


    std::string youBotChildFrameID;
    std::string youBotOdometryFrameID;
    std::string youBotOdometryChildFrameID;
    std::string youBotArmFrameID;


    /// ROS timestamp
    rclcpp::Time currentTime;


    /// The published odometry message with distances in [m], angles in [RAD] and velocities in [m/s] and [RAD/s]
    nav_msgs::msg::Odometry odometryMessage;

    /// The published odometry tf frame with distances in [m]
    geometry_msgs::msg::TransformStamped odometryTransform;

    /// The quaternion inside the tf odometry frame with distances in [m]
    geometry_msgs::msg::Quaternion odometryQuaternion;

    /// The published joint state of the base (wheels) with angles in [RAD] and velocities in [RAD/s]
    sensor_msgs::msg::JointState baseJointStateMessage;

    /// Vector of the published joint states of per arm with angles in [RAD]
    vector<sensor_msgs::msg::JointState> armJointStateMessages;

    /// The joint trajectory goal that is currently active.
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory> > armActiveJointTrajectoryGoal;

    /// Tell if a goal is currently active.
    bool armHasActiveJointTrajectoryGoal;

    youbot::GripperSensedBarPosition gripperBar1Position;
    youbot::GripperSensedBarPosition gripperBar2Position;

    //void executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal,  int armIndex);
    
    //bool trajectoryActionServerEnable;
    //double trajectoryVelocityGain;
    //double trajectoryPositionGain;
    double youBotDriverCycleFrequencyInHz;
    double youBotDriverGripperReadingsCycleFrequencyInHz;
        
    /// diagnostic msgs
    rclcpp::Time lastDiagnosticPublishTime;

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticArrayPublisher;
    diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMessage;
    diagnostic_msgs::msg::DiagnosticStatus diagnosticStatusMessage;
    std::string diagnosticNameArm;
    std::string diagnosticNameBase;

    bool areBaseMotorsSwitchedOn;
    bool areArmMotorsSwitchedOn;

    rclcpp::Time last_gripper_readings_time_;
};

} // namespace youBot

#endif /* YOUBOTOODLWRAPPER_H_ */

/* EOF */
