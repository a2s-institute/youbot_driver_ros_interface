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

#ifndef YOUBOTCONFIGURATION_H_
#define YOUBOTCONFIGURATION_H_

/* ROS includes */
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "brics_actuator/msg/joint_positions.hpp"
#include "brics_actuator/msg/joint_velocities.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

/* OODL includes */
#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>

class JointTrajectoryAction;

namespace youBot
{

class YouBotBaseConfiguration
{
public:

    /// Standard constructor
    YouBotBaseConfiguration();

    /// Standard destructor
    virtual ~YouBotBaseConfiguration();

    /// Handle to the base
    youbot::YouBotBase* youBotBase;


    /// "Name" of the base. Typically derived from name of youBot configuration file.
    std::string baseID;

    /// Joint names for the wheels
    std::vector<std::string> wheelNames;


    /// Receives Twist messages for the base.
    rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr baseCommandSubscriber;


    /// Publishes Odometry messages
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr baseOdometryPublisher;

    /// Publishes JointState messages with angles/velocities for the wheels.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr baseJointStatePublisher;

    /// Service to switch the motor off by setting the PWM value to zero
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switchOffMotorsService;

    /// Service to switch the motor ON by setting the velocity to zero
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switchONMotorsService;

    /// Publishes tf frames as odometry
    std::shared_ptr<tf2_ros::TransformBroadcaster> odometryBroadcaster;


};

class YouBotArmConfiguration
{
public:

    /// Standard constructor
    YouBotArmConfiguration();

    /// Standard destructor
    virtual ~YouBotArmConfiguration();

    /// Handle to the arm
    youbot::YouBotManipulator* youBotArm;


    /// "Name" of the arm. Typically derived from name of youBot configuration file.
    std::string armID;

    /// Name prefix for topic e.g arm_1/...
    std::string commandTopicName;

    /// Parent frameID for the published messages
    std::string parentFrameIDName;

    /// Names of the joints. Are typically derived from the configuration files
    std::vector<std::string> jointNames;

    ///Joint names for the gripper fingers
    std::vector<std::string> gripperFingerNames;

    const static unsigned int LEFT_FINGER_INDEX = 0;
    const static unsigned int RIGHT_FINGER_INDEX = 1;

    /// Receives "brics_actuator/JointPositions" for the arm joints
    rclcpp::Subscription<brics_actuator::msg::JointPositions>::ConstSharedPtr armPositionCommandSubscriber;

    /// Receives "brics_actuator/JointVelocities" for the arm joints
    rclcpp::Subscription<brics_actuator::msg::JointVelocities>::ConstSharedPtr armVelocityCommandSubscriber;

	/// Implements a "control_msgs/FollowJointTrajectory" action
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr armJointTrajectoryAction;

    /// Receives "brics_actuator/JointPositions" for the gripper
    rclcpp::Subscription<brics_actuator::msg::JointPositions>::ConstSharedPtr gripperPositionCommandSubscriber;


    /// Publishes JointState messages with angles for the arm.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr armJointStatePublisher;

    /// Service to switch the motor off by setting the PWM value to zero
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switchOffMotorsService;

    /// Service to switch the motor ON by setting the velocity to zero
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switchONMotorsService;

    /// Service to calibrate the arm
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calibrateService;

    //Server* trajectoryActionServer;
    //JointTrajectoryAction* jointTrajectoryAction;

    /**
     * This variable memorizes the last successfully set value for the gripper,
     * so it can be published in the joint state message. This is necessary at the moment, as
     * it is not yet possible to measure the actual distance. Consider the gripper joint state
     * as an open loop value.
     */
    double lastGripperCommand;

};

/**
 * @brief Aggregation class for instantiated parts of a youBot systems and all its name mapping between ROS and the youBot API.
 */
class YouBotConfiguration
{
public:
    YouBotConfiguration();
    virtual ~YouBotConfiguration();

    /// Path to the configuration files, required by OODL (e.g. youbot-base.cfg)
    std::string configurationFilePath;

    ///Flag to indicate if youBot has a base (set after successful initialization)
    bool hasBase;

    ///Flag to indicate if youBot has one or more arms (set after successful initialization)
    bool hasArms;

    /// A youbot system has one base
    YouBotBaseConfiguration baseConfiguration;

    /// A youbot system has one or more arms
    std::vector<YouBotArmConfiguration> youBotArmConfigurations;
    std::map<std::string, int> armNameToArmIndexMapping;
    
    /// Publishes diagnostic messages
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticArrayPublisher;
};




} // namespace youBot

#endif /* YOUBOTCONFIGURATION_H_ */

/* EOF */
