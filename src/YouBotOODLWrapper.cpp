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

#include "youbot_driver_ros_interface/YouBotOODLWrapper.h"
#include "youbot_driver_ros_interface/joint_state_observer_oodl.h"

#include <youbot_trajectory_action_server/joint_trajectory_action.h>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace youBot
{

YouBotOODLWrapper::YouBotOODLWrapper()
{
}

YouBotOODLWrapper::YouBotOODLWrapper(rclcpp::Node::SharedPtr node) : node(node)
{

    youBotConfiguration.hasBase = false;
    youBotConfiguration.hasArms = false;
    areBaseMotorsSwitchedOn = false;	
    areArmMotorsSwitchedOn = false;

    youBotConfiguration.baseConfiguration.odometryBroadcaster.reset(new tf2_ros::TransformBroadcaster(node));

    youBotChildFrameID = "base_link"; //holds true for both: base and arm
    armJointStateMessages.clear();

    node->get_parameter("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz);
    node->get_parameter("youBotDriverGripperReadingsCycleFrequencyInHz", youBotDriverGripperReadingsCycleFrequencyInHz);
    diagnosticNameArm = "platform_Arm";
    diagnosticNameBase = "platform_Base";
    diagnosticArrayPublisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

    last_gripper_readings_time_ = node->now();
}

YouBotOODLWrapper::~YouBotOODLWrapper()
{
    this->stop();
    diagnosticArrayPublisher.reset();
}

void YouBotOODLWrapper::initializeBase(std::string baseName)
{

    try
    {
        youBotConfiguration.baseConfiguration.youBotBase = new youbot::YouBotBase(baseName, youBotConfiguration.configurationFilePath);
        youBotConfiguration.baseConfiguration.youBotBase->doJointCommutation();
    }
    catch (std::exception& e)
    {
        std::string errorMessage = e.what();
        RCLCPP_FATAL(this->node->get_logger(), "%s", errorMessage.c_str());
        RCLCPP_ERROR(this->node->get_logger(), "Base \"%s\" could not be initialized.", baseName.c_str());
        youBotConfiguration.hasBase = false;
        return;
    }

    /* setup input/output communication */
    youBotConfiguration.baseConfiguration.baseCommandSubscriber = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1000, std::bind(&YouBotOODLWrapper::baseCommandCallback, this, std::placeholders::_1));
    youBotConfiguration.baseConfiguration.baseOdometryPublisher = node->create_publisher<nav_msgs::msg::Odometry > ("odom", 1);
    youBotConfiguration.baseConfiguration.baseJointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState > ("base/joint_states", 1);

    /* setup services*/
    youBotConfiguration.baseConfiguration.switchOffMotorsService = node->create_service<std_srvs::srv::Empty>("base/switchOffMotors", std::bind(&YouBotOODLWrapper::switchOffBaseMotorsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    youBotConfiguration.baseConfiguration.switchONMotorsService = node->create_service<std_srvs::srv::Empty>("base/switchOnMotors", std::bind(&YouBotOODLWrapper::switchOnBaseMotorsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    /* setup frame_ids */
    youBotOdometryFrameID = "odom";
    youBotOdometryChildFrameID = "base_footprint";

    RCLCPP_INFO(node->get_logger(), "Base is initialized.");
    youBotConfiguration.hasBase = true;
    areBaseMotorsSwitchedOn = true;
}

void YouBotOODLWrapper::initializeArm(std::string armName)
{
    int armIndex;
    youbot::JointName jointNameParameter;
    std::string jointName;
    stringstream topicName;
    stringstream serviceName;

    try
    {
        YouBotArmConfiguration tmpArmConfig;
        youBotConfiguration.youBotArmConfigurations.push_back(tmpArmConfig);
        armIndex = static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()) - 1;
        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm = new youbot::YouBotManipulator(armName, youBotConfiguration.configurationFilePath);
        youBotConfiguration.youBotArmConfigurations[armIndex].armID = armName;
        topicName.str("");
        topicName << "arm_" << (armIndex + 1) << "/";
        youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName = topicName.str(); // e.g. arm_1/
        youBotConfiguration.youBotArmConfigurations[armIndex].parentFrameIDName = "base_link";
        youBotConfiguration.armNameToArmIndexMapping.insert(make_pair(armName, static_cast<int> (youBotConfiguration.youBotArmConfigurations.size())));

        /* take joint names form configuration files */
        youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.clear();
        for (int i = 0; i < youBotArmDoF; ++i)
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getConfigurationParameter(jointNameParameter);
            jointNameParameter.getParameter(jointName);
            youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.push_back(jointName);
            RCLCPP_INFO(node->get_logger(), "Joint %i for arm %s has name: %s", i + 1, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i].c_str());

        }


        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->doJointCommutation();
        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateManipulator();
    }
    catch (std::exception& e)
    {
        youBotConfiguration.youBotArmConfigurations.pop_back();
        std::string errorMessage = e.what();
        RCLCPP_FATAL(node->get_logger(), "%s", errorMessage.c_str());
        RCLCPP_ERROR(node->get_logger(), "Arm \"%s\" could not be initialized.", armName.c_str());
        RCLCPP_INFO(node->get_logger(), "System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
        return;
    }

    if(youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->hasGripper())
    {
        try
        {
            youbot::GripperBarName barName;
            std::string gripperBarName;

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX] = gripperBarName;
            RCLCPP_INFO(node->get_logger(), "Joint %i for gripper of arm %s has name: %s", 1, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), gripperBarName.c_str());

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX] = gripperBarName;
            RCLCPP_INFO(node->get_logger(), "Joint %i for gripper of arm %s has name: %s", 2, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), gripperBarName.c_str());

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateGripper();
        }
        catch (std::exception& e)
        {
            RCLCPP_WARN_STREAM(node->get_logger(), "Gripper on arm \"" << armName << "\" could not be initialized: " << e.what());
        }
    }
    else
    {
        RCLCPP_WARN_STREAM(node->get_logger(), "Gripper on arm \"" << armName << "\" not found or disabled in the config file!");
    }



    /* setup input/output communication */
    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/position_command"; // e.g. arm_1/arm_controller/positionCommand
    std::function<void(brics_actuator::msg::JointPositions::SharedPtr)> joint_pos_fn = std::bind(&YouBotOODLWrapper::armPositionsCommandCallback, this, std::placeholders::_1, armIndex);
    youBotConfiguration.youBotArmConfigurations[armIndex].armPositionCommandSubscriber = node->create_subscription<brics_actuator::msg::JointPositions > (topicName.str(), 1000, joint_pos_fn);

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/velocity_command";
    std::function<void(brics_actuator::msg::JointVelocities::SharedPtr)> joint_vel_fn = std::bind(&YouBotOODLWrapper::armVelocitiesCommandCallback, this, std::placeholders::_1, armIndex);
    youBotConfiguration.youBotArmConfigurations[armIndex].armVelocityCommandSubscriber = node->create_subscription<brics_actuator::msg::JointVelocities > (topicName.str(), 1000, joint_vel_fn);

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/follow_joint_trajectory";
    // topicName.str("/arm_1/arm_controller/follow_joint_trajectory");
    std::function<rclcpp_action::GoalResponse(const rclcpp_action::GoalUUID,
                       std::shared_ptr<const FollowJointTrajectory::Goal>)> goal_callback_fn = std::bind(&YouBotOODLWrapper::armJointTrajectoryGoalCallback, this, std::placeholders::_1, std::placeholders::_2, armIndex);
    std::function<rclcpp_action::CancelResponse(const std::shared_ptr<GoalHandleFollowJointTrajectory>)> cancel_callback_fn = std::bind(&YouBotOODLWrapper::armJointTrajectoryCancelCallback, this, std::placeholders::_1, armIndex);
    std::function<void(const std::shared_ptr<GoalHandleFollowJointTrajectory>)> accept_callback_fn = std::bind(&YouBotOODLWrapper::armJointTrajectoryAcceptCallback, this, std::placeholders::_1, armIndex);
    youBotConfiguration.youBotArmConfigurations[armIndex].armJointTrajectoryAction = rclcpp_action::create_server<FollowJointTrajectory>(
                                                                        node,
                                                                        topicName.str(),
                                                                        goal_callback_fn,
                                                                        cancel_callback_fn,
                                                                        accept_callback_fn);

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "joint_states";
    youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState > (topicName.str(), 1); //TODO different names or one topic?

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "gripper_controller/position_command";
    joint_pos_fn = std::bind(&YouBotOODLWrapper::gripperPositionsCommandCallback, this, std::placeholders::_1, armIndex);
    youBotConfiguration.youBotArmConfigurations[armIndex].gripperPositionCommandSubscriber = node->create_subscription<brics_actuator::msg::JointPositions > (topicName.str(), 1000, joint_pos_fn);
    youBotConfiguration.youBotArmConfigurations[armIndex].lastGripperCommand = 0.0; //This is true if the gripper is calibrated.

    /* setup services*/
    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "switchOffMotors"; // e.g. "arm_1/switchOffMotors"
    std::function<void(const std::shared_ptr<rmw_request_id_t>,
                       const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>)> arm_service_callback_fn = std::bind(&YouBotOODLWrapper::switchOffArmMotorsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, armIndex);

    youBotConfiguration.youBotArmConfigurations[armIndex].switchOffMotorsService = node->create_service<std_srvs::srv::Empty> (serviceName.str(), arm_service_callback_fn);

    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "switchOnMotors"; // e.g. "arm_1/switchOnMotors"
    arm_service_callback_fn  = std::bind(&YouBotOODLWrapper::switchOnArmMotorsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, armIndex);
    youBotConfiguration.youBotArmConfigurations[armIndex].switchONMotorsService = node->create_service<std_srvs::srv::Empty > (serviceName.str(), arm_service_callback_fn);

    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "calibrate"; // e.g. "arm_1/calibrate"
    arm_service_callback_fn = std::bind(&YouBotOODLWrapper::calibrateArmCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, armIndex);
    youBotConfiguration.youBotArmConfigurations[armIndex].calibrateService = node->create_service<std_srvs::srv::Empty > (serviceName.str(), arm_service_callback_fn);
/*
    if (trajectoryActionServerEnable)
    {
        JointStateObserver* jointStateObserver = new JointStateObserverOODL(this, armIndex);
        topicName.str("");
        topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "action";
        youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction = new JointTrajectoryAction(jointStateObserver,
                                                                                                                trajectoryPositionGain,
                                                                                                                trajectoryVelocityGain,
                                                                                                                youBotDriverCycleFrequencyInHz);

        serviceName.str("");
        serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/joint_trajectory_action"; // e.g. "arm_1/switchOnMotors"
        
        youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer = new Server(serviceName.str(),
                                                                                                  boost::bind(&YouBotOODLWrapper::executeActionServer, this, _1, armIndex),
                                                                                                  false);
        youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer->start();
    }
*/
    /* initialize message vector for arm joint states */
    sensor_msgs::msg::JointState dummyMessage;
    armJointStateMessages.push_back(dummyMessage);

    /* setup frame_ids */
    youBotArmFrameID = "arm"; //TODO find default topic name
    RCLCPP_INFO(node->get_logger(), "Arm \"%s\" is initialized.", armName.c_str());
    RCLCPP_INFO(node->get_logger(), "System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
    youBotConfiguration.hasArms = true;
    areArmMotorsSwitchedOn = true;

    // currently no action is running
    armHasActiveJointTrajectoryGoal = false;

    //tracejoint = 4;
    //myTrace = new youbot::DataTrace(youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(tracejoint), "Joint4TrajectoryTrace");

    // we can handle actionlib requests only after the complete initialization has been performed
    // TODO: is there an equivalent in ROS2?
    //youBotConfiguration.youBotArmConfigurations[armIndex].armJointTrajectoryAction->start();
}

/*
void YouBotOODLWrapper::executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, int armIndex)
{

    JointTrajectoryAction* jointTrajectoryAction = youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction;
    if (jointTrajectoryAction != NULL)
    {

        jointTrajectoryAction->execute(goal, youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer);
    }
}
*/
void YouBotOODLWrapper::stop()
{

    if (youBotConfiguration.baseConfiguration.youBotBase)
    {
        delete youBotConfiguration.baseConfiguration.youBotBase;
        youBotConfiguration.baseConfiguration.youBotBase = 0;
    }

    youBotConfiguration.baseConfiguration.baseCommandSubscriber.reset();
    youBotConfiguration.baseConfiguration.baseJointStatePublisher.reset();
    youBotConfiguration.baseConfiguration.baseOdometryPublisher.reset();
    youBotConfiguration.baseConfiguration.switchONMotorsService.reset();
    youBotConfiguration.baseConfiguration.switchOffMotorsService.reset();
    // youBotConfiguration.baseConfiguration.odometryBroadcaster.
    youBotConfiguration.hasBase = false;
    areBaseMotorsSwitchedOn = false;


    for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++) //delete each arm
    {
        if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm)
        {
            delete youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm;
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm = 0;
        }

        youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher.reset();
        youBotConfiguration.youBotArmConfigurations[armIndex].armPositionCommandSubscriber.reset();
        youBotConfiguration.youBotArmConfigurations[armIndex].armVelocityCommandSubscriber.reset();
        youBotConfiguration.youBotArmConfigurations[armIndex].calibrateService.reset();
        youBotConfiguration.youBotArmConfigurations[armIndex].gripperPositionCommandSubscriber.reset();
        youBotConfiguration.youBotArmConfigurations[armIndex].switchONMotorsService.reset();
        youBotConfiguration.youBotArmConfigurations[armIndex].switchOffMotorsService.reset();
    }

    youBotConfiguration.hasArms = false;
    areArmMotorsSwitchedOn = false;
    youBotConfiguration.youBotArmConfigurations.clear();
    armJointStateMessages.clear();

    youbot::EthercatMaster::destroy();
}

void YouBotOODLWrapper::baseCommandCallback(geometry_msgs::msg::Twist::SharedPtr youbotBaseCommand)
{

    if (youBotConfiguration.hasBase)
    { // in case stop has been invoked
        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        /*
         * Frame in OODL:
         *
         *         FRONT
         *
         *         X
         *         ^
         *         |
         *         |
         *         |
         * Y <-----+
         *
         *        BACK
         *
         * Positive angular velocity means turning counterclockwise
         *
         */

        longitudinalVelocity = youbotBaseCommand->linear.x * meter_per_second;
        transversalVelocity = youbotBaseCommand->linear.y * meter_per_second;
        angularVelocity = youbotBaseCommand->angular.z * radian_per_second;

        try
        {
            youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            RCLCPP_WARN(node->get_logger(), "Cannot set base velocities: %s", errorMessage.c_str());
        }

    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "No base initialized!");
    }
}

void YouBotOODLWrapper::armPositionsCommandCallback(brics_actuator::msg::JointPositions::SharedPtr youbotArmCommand, int armIndex)
{
    RCLCPP_DEBUG(node->get_logger(), "Command for arm%i received", armIndex + 1);
    assert(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) // in case stop has been invoked
    {

        RCLCPP_DEBUG(node->get_logger(), "Arm ID is: %s", youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str());
        if (youbotArmCommand->positions.size() < 1)
        {
            RCLCPP_WARN(node->get_logger(), "youBot driver received an invalid joint positions command.");
            return;
        }

        youbot::JointAngleSetpoint desiredAngle;
        string unit = boost::units::to_string(boost::units::si::radian);

        /* populate mapping between joint names and values  */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->positions.size()); ++i)
        {
            if (unit == youbotArmCommand->positions[i].unit)
            {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->positions[i].joint_uri, youbotArmCommand->positions[i].value));
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->positions[i].unit.c_str(), unit.c_str());
            }
        }

        /* loop over all youBot arm joints and check if something is in the received message that requires action */
        assert(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));

        try
        {
            youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time

            for (int i = 0; i < youBotArmDoF; ++i)
            {

                /* check what is in map */
                map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
                if (jointIterator != jointNameToValueMapping.end())
                {

                    /* set the desired joint value */
                    RCLCPP_DEBUG(node->get_logger(), "Trying to set joint %s to new position value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                    desiredAngle.angle = jointIterator->second * radian;
                    try
                    {
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredAngle); //youBot joints start with 1 not with 0 -> i + 1
                    }
                    catch (std::exception& e)
                    {
                        std::string errorMessage = e.what();
                        RCLCPP_WARN(node->get_logger(), "Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
                    }
                }
            }
            youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
        } catch(std::exception &e)
        {
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Arm%i is not correctly initialized!", armIndex + 1);
    }

}

void YouBotOODLWrapper::armVelocitiesCommandCallback(brics_actuator::msg::JointVelocities::SharedPtr youbotArmCommand, int armIndex)
{
    RCLCPP_DEBUG(node->get_logger(), "Command for arm%i received", armIndex + 1);
    // TODO
    assert(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    { // in case stop has been invoked


        if (youbotArmCommand->velocities.size() < 1)
        {
            RCLCPP_WARN(node->get_logger(), "youBot driver received an invalid joint velocities command.");
            return;
        }

        youbot::JointVelocitySetpoint desiredAngularVelocity;
        string unit = boost::units::to_string(boost::units::si::radian_per_second);

        /* populate mapping between joint names and values  */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->velocities.size()); ++i)
        {
            if (unit == youbotArmCommand->velocities[i].unit)
            {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->velocities[i].joint_uri, youbotArmCommand->velocities[i].value));
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->velocities[i].unit.c_str(), unit.c_str());
            }

        }

        /* loop over all youBot arm joints and check if something is in the received message that requires action */
        // TODO
        assert(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));

        try
        {
            youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
            for (int i = 0; i < youBotArmDoF; ++i)
            {

                /* check what is in map */
                map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
                if (jointIterator != jointNameToValueMapping.end())
                {

                    /* set the desired joint value */
                    RCLCPP_DEBUG(node->get_logger(), "Trying to set joint %s to new velocity value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                    desiredAngularVelocity.angularVelocity = jointIterator->second * radian_per_second;
                    try
                    {
                        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredAngularVelocity); //youBot joints start with 1 not with 0 -> i + 1

                    }
                    catch (std::exception& e)
                    {
                        std::string errorMessage = e.what();
                        RCLCPP_WARN(node->get_logger(), "Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
                    }
                }
            }
            youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
        } catch(std::exception &e)
        {
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Arm%i is not correctly initialized!", armIndex + 1);
    }
}


rclcpp_action::GoalResponse YouBotOODLWrapper::armJointTrajectoryGoalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> youbotArmGoal, unsigned int armIndex) {
    RCLCPP_INFO(node->get_logger(), "Goal for arm%i received", armIndex + 1);
    assert(armIndex < youBotConfiguration.youBotArmConfigurations.size());

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm == 0) {
        RCLCPP_ERROR(node->get_logger(), "Arm%i is not correctly initialized!", armIndex + 1);
        return rclcpp_action::GoalResponse::REJECT;
    }


    trajectory_msgs::msg::JointTrajectory trajectory = youbotArmGoal->trajectory;

    // validate that the correct number of joints is provided in the goal
    if (trajectory.joint_names.size() != static_cast<unsigned int> (youBotArmDoF)) {
        RCLCPP_ERROR(node->get_logger(), "Trajectory is malformed! Goal has %i joint names, but only %i joints are supported", static_cast<int> (trajectory.joint_names.size()), youBotArmDoF);
        return rclcpp_action::GoalResponse::REJECT;
    }

    // compare the joint names of the youBot configuration and joint names provided with the trajectory
    for (unsigned int i = 0; i < youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size(); i++) {
        bool jointNameFound = false;
        for (unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
            if (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i] == trajectory.joint_names[j]) {
                jointNameFound = true;
                break;
            }
        }

        if (!jointNameFound) {
            RCLCPP_ERROR(node->get_logger(), "Trajectory is malformed! Joint %s is missing in the goal", youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i].c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // verify that the trajectory points have as many fields as DoF of the arm
    for (unsigned int i = 0; i < trajectory.points.size(); i++) {
        trajectory_msgs::msg::JointTrajectoryPoint point = trajectory.points[i];
        // validate the trajectory point
        if ((point.positions.size() != static_cast<unsigned int> (youBotArmDoF)
                        || point.velocities.size() != static_cast<unsigned int> (youBotArmDoF)
                        || point.accelerations.size() != static_cast<unsigned int> (youBotArmDoF))) {
            RCLCPP_ERROR(node->get_logger(), "A trajectory point is malformed! %i positions, velocities and accelerations must be provided", youBotArmDoF);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YouBotOODLWrapper::armJointTrajectoryCancelCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> youbotArmGoal, unsigned int armIndex) {
    RCLCPP_DEBUG(node->get_logger(), "Cancel goal for arm%i received", armIndex + 1);
    assert(armIndex < youBotConfiguration.youBotArmConfigurations.size());

    // stop the controller
    for (int i = 0; i < youBotArmDoF; ++i) {
        try {
            // youBot joints start with 1 not with 0 -> i + 1
      //TODO cancel trajectory
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.cancelCurrentTrajectory();
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).stopJoint();
        } catch (std::exception& e) {
            std::string errorMessage = e.what();
            RCLCPP_WARN(node->get_logger(), "Cannot stop joint %i: %s", i + 1, errorMessage.c_str());
        }
    }

    if (armActiveJointTrajectoryGoal == youbotArmGoal) {
        armHasActiveJointTrajectoryGoal = false;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void YouBotOODLWrapper::armJointTrajectoryAcceptCallback(const std::shared_ptr<GoalHandleFollowJointTrajectory> youbotArmGoal, unsigned int armIndex)
{
    using namespace std::placeholders;  // NOLINT
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&YouBotOODLWrapper::armJointTrajectoryExecute, this, std::placeholders::_1, std::placeholders::_2), youbotArmGoal, armIndex}.detach();
}

void YouBotOODLWrapper::armJointTrajectoryExecute(const std::shared_ptr<GoalHandleFollowJointTrajectory> youbotArmGoal, unsigned int armIndex)
{
    trajectory_msgs::msg::JointTrajectory trajectory = youbotArmGoal->get_goal()->trajectory;

    std::vector<youbot::JointTrajectory> jointTrajectories(youBotArmDoF);

    // convert from the ROS trajectory representation to the controller's representation
    std::vector<std::vector< quantity<plane_angle> > > positions(youBotArmDoF);
    std::vector<std::vector< quantity<angular_velocity> > > velocities(youBotArmDoF);
    std::vector<std::vector< quantity<angular_acceleration> > > accelerations(youBotArmDoF);
    youbot::TrajectorySegment segment;
    for (unsigned int i = 0; i < trajectory.points.size(); i++) {
        trajectory_msgs::msg::JointTrajectoryPoint point = trajectory.points[i];
        for (int j = 0; j < youBotArmDoF; j++) {
            segment.positions = point.positions[j]*radian;
            segment.velocities = point.velocities[j]*radian_per_second;
            segment.accelerations = point.accelerations[j] * radian_per_second/second;
            rclcpp::Duration time_from_start(point.time_from_start);
            segment.time_from_start = boost::posix_time::microsec(time_from_start.nanoseconds()/1000);
            jointTrajectories[j].segments.push_back(segment);
        }
    }
    for (int j = 0; j < youBotArmDoF; j++) {
        jointTrajectories[j].start_time = boost::posix_time::microsec_clock::local_time(); //TODO is this correct to set the trajectory start time to now
    }


    // cancel the old goal
  /*
    if (armHasActiveJointTrajectoryGoal) {
        armActiveJointTrajectoryGoal.setCanceled();
        armHasActiveJointTrajectoryGoal = false;
        for (int i = 0; i < youBotArmDoF; ++i) {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).cancelTrajectory();
        }
    }
  */

    armActiveJointTrajectoryGoal = youbotArmGoal;
    armHasActiveJointTrajectoryGoal = true;

    // send the trajectory to the controller
    for (int i = 0; i < youBotArmDoF; ++i) {
        try {
            // youBot joints start with 1 not with 0 -> i + 1
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.setTrajectory(jointTrajectories[i]);
      RCLCPP_INFO(node->get_logger(), "set trajectories %d", i);
        } catch (std::exception& e) {
            std::string errorMessage = e.what();
            RCLCPP_WARN(node->get_logger(), "Cannot set trajectory for joint %i: %s", i + 1, errorMessage.c_str());
        }
    }
    RCLCPP_INFO(node->get_logger(), "set all trajectories");
}

void YouBotOODLWrapper::gripperPositionsCommandCallback(brics_actuator::msg::JointPositions::SharedPtr youbotGripperCommand, int armIndex)
{
    RCLCPP_DEBUG(node->get_logger(), "Command for gripper%i received", armIndex + 1);
    assert(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) { // in case stop has been invoked

        if (youbotGripperCommand->positions.size() < 1){
            RCLCPP_WARN(node->get_logger(), "youBot driver received an invalid gripper positions command.");
            return;
        }

        map<string, double>::const_iterator gripperIterator;
        youbot::GripperBarPositionSetPoint leftGripperFingerPosition;
        youbot::GripperBarPositionSetPoint rightGripperFingerPosition;
        string unit = boost::units::to_string(boost::units::si::meter);

        /* populate mapping between joint names and values */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int>(youbotGripperCommand->positions.size()); ++i) {
            if (unit == youbotGripperCommand->positions[i].unit) {
                jointNameToValueMapping.insert(make_pair(youbotGripperCommand->positions[i].joint_uri, youbotGripperCommand->positions[i].value));
            } else {
                RCLCPP_WARN(node->get_logger(), "Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotGripperCommand->positions[i].unit.c_str(), unit.c_str());
            }
        }

        try
        {
            youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time

            /* check if something is in the received message that requires action for the left finger gripper */
            gripperIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX]);
            if (gripperIterator != jointNameToValueMapping.end()) {
                RCLCPP_DEBUG(node->get_logger(), "Trying to set the left gripper finger to new value %f", gripperIterator->second);

                leftGripperFingerPosition.barPosition = gripperIterator->second * meter;
                try {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1().setData(leftGripperFingerPosition);
                } catch (std::exception& e) {
                    std::string errorMessage = e.what();
                    RCLCPP_WARN(node->get_logger(), "Cannot set the left gripper finger: %s", errorMessage.c_str());
                }
            }

            /* check if something is in the received message that requires action for the right finger gripper */
            gripperIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX]);
            if (gripperIterator != jointNameToValueMapping.end()) {
                RCLCPP_DEBUG(node->get_logger(), "Trying to set the right gripper to new value %f", gripperIterator->second);

                rightGripperFingerPosition.barPosition = gripperIterator->second * meter;
                try {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2().setData(rightGripperFingerPosition);
                } catch (std::exception& e) {
                    std::string errorMessage = e.what();
                    RCLCPP_WARN(node->get_logger(), "Cannot set the right gripper finger: %s", errorMessage.c_str());
                }
            }

            youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
        } catch(std::exception &e)
        {
            return;
        }

    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm%i is not correctly initialized!", armIndex + 1);
    }
}

void YouBotOODLWrapper::computeOODLSensorReadings()
{

    try{
    currentTime = node->now();
    youbot::JointSensedAngle currentAngle;
    youbot::JointSensedVelocity currentVelocity;
    youbot::JointSensedTorque currentTorque;

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false); // ensure that all joint values will be received at the same time

    if (youBotConfiguration.hasBase == true)
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

        double vx = 0.0;
        double vy = 0.0;
        double vtheta = 0.0;

        quantity<si::length> longitudinalPosition;
        quantity<si::length> transversalPosition;
        quantity<plane_angle> orientation;

        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        youBotConfiguration.baseConfiguration.youBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
        x = longitudinalPosition.value();
        y = transversalPosition.value();
        theta = orientation.value();

        youBotConfiguration.baseConfiguration.youBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        vx = longitudinalVelocity.value();
        vy = transversalVelocity.value();
        vtheta = angularVelocity.value();
        //RCLCPP_DEBUG(node->get_logger(), "Perceived odometric values (x,y,tetha, vx,vy,vtetha): %f, %f, %f \t %f, %f, %f", x, y, theta, vx, vy, vtheta);


        /* Setup odometry tf frame */
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, theta);
        tf2::convert(quat_tf, odometryQuaternion);

        odometryTransform.header.stamp = currentTime;
        odometryTransform.header.frame_id = youBotOdometryFrameID;
        odometryTransform.child_frame_id = youBotOdometryChildFrameID;

        odometryTransform.transform.translation.x = x;
        odometryTransform.transform.translation.y = y;
        odometryTransform.transform.translation.z = 0.0;
        odometryTransform.transform.rotation = odometryQuaternion;

        /* Setup odometry Message */
        odometryMessage.header.stamp = currentTime;
        odometryMessage.header.frame_id = youBotOdometryFrameID;

        odometryMessage.pose.pose.position.x = x;
        odometryMessage.pose.pose.position.y = y;
        odometryMessage.pose.pose.position.z = 0.0;
        odometryMessage.pose.pose.orientation = odometryQuaternion;

        odometryMessage.child_frame_id = youBotOdometryChildFrameID;
        //        odometryMessage.child_frame_id = youBotOdometryFrameID;
        odometryMessage.twist.twist.linear.x = vx;
        odometryMessage.twist.twist.linear.y = vy;
        odometryMessage.twist.twist.angular.z = vtheta;

        /* Set up joint state message for the wheels */
        baseJointStateMessage.header.stamp = currentTime;
        baseJointStateMessage.name.resize(youBotNumberOfWheels * 2); // *2 because of virtual wheel joints in the URDF description
        baseJointStateMessage.position.resize(youBotNumberOfWheels * 2);
        baseJointStateMessage.velocity.resize(youBotNumberOfWheels * 2);

        assert((youBotConfiguration.baseConfiguration.wheelNames.size() == static_cast<unsigned int> (youBotNumberOfWheels)));
        for (int i = 0; i < youBotNumberOfWheels; ++i)
        {
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentVelocity);

            baseJointStateMessage.name[i] = youBotConfiguration.baseConfiguration.wheelNames[i];
            baseJointStateMessage.position[i] = currentAngle.angle.value();
            baseJointStateMessage.velocity[i] = currentVelocity.angularVelocity.value();
        }

        /*
         * Here we add values for "virtual" rotation joints in URDF - robot_state_publisher can't
         * handle non-aggregated jointState messages well ...
         */
        baseJointStateMessage.name[4] = "caster_joint_fl";
        baseJointStateMessage.position[4] = 0.0;

        baseJointStateMessage.name[5] = "caster_joint_fr";
        baseJointStateMessage.position[5] = 0.0;

        baseJointStateMessage.name[6] = "caster_joint_bl";
        baseJointStateMessage.position[6] = 0.0;

        baseJointStateMessage.name[7] = "caster_joint_br";
        baseJointStateMessage.position[7] = 0.0;

        /*
         * Yet another hack to make the published values compatible with the URDF description.
         * We actually flipp the directions of the wheel on the right side such that the standard ROS controllers
         * (e.g. for PR2) can be used for the youBot
         */
        baseJointStateMessage.position[0] = -baseJointStateMessage.position[0];
        baseJointStateMessage.position[2] = -baseJointStateMessage.position[2];

    }

    if (youBotConfiguration.hasArms == true)
    {

        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++)
        {
            assert(youBotConfiguration.youBotArmConfigurations.size() == armJointStateMessages.size());

            /* fill joint state message */
            armJointStateMessages[armIndex].header.stamp = currentTime;

            double numberOfArmjoints = 0;
            if(youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->hasGripper())
                numberOfArmjoints = youBotArmDoF + youBotNumberOfFingers;
            else
                numberOfArmjoints = youBotArmDoF;

            armJointStateMessages[armIndex].name.resize(numberOfArmjoints);
            armJointStateMessages[armIndex].position.resize(numberOfArmjoints);
            armJointStateMessages[armIndex].velocity.resize(numberOfArmjoints);
            armJointStateMessages[armIndex].effort.resize(youBotArmDoF + youBotNumberOfFingers);

            if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm == 0)
            {
                RCLCPP_ERROR(node->get_logger(), "Arm%i is not correctly initialized! Cannot publish data.", armIndex + 1);
                continue;
            }

            assert(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
            for (int i = 0; i < youBotArmDoF; ++i)
            {
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1 //FIXME might segfault if only 1eout of 2 arms are initialized.
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentVelocity);
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentTorque);

                armJointStateMessages[armIndex].name[i] = youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]; //TODO no unique names for URDF yet
                armJointStateMessages[armIndex].position[i] = currentAngle.angle.value();
                armJointStateMessages[armIndex].velocity[i] = currentVelocity.angularVelocity.value();
                armJointStateMessages[armIndex].effort[i] = currentTorque.torque.value();
            }

            // check if trajectory controller is finished
            bool areTrajectoryControllersDone = true;
            for (int i = 0; i < youBotArmDoF; ++i) {
                if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.isTrajectoryControllerActive()) {
                    areTrajectoryControllersDone = false;
                    break;
                }
            }
            if (areTrajectoryControllersDone && armHasActiveJointTrajectoryGoal) {
                armHasActiveJointTrajectoryGoal = false;
                auto trajectoryResult = std::make_shared<FollowJointTrajectory::Result>();
                trajectoryResult->error_code = trajectoryResult->SUCCESSFUL;
                armActiveJointTrajectoryGoal->succeed(trajectoryResult);
                // RCLCPP_INFO(node->get_logger(), "trajectory successful");
                // myTrace->stopTrace();
                // myTrace->plotTrace();
			}

            /*
             * NOTE: gripper slide rails are always symmetric, but the fingers can be screwed in different
             * positions! The published values account for the distance between the gripper slide rails, not the fingers
             * themselves. Of course if the finger are screwed to the most inner position (i.e. the can close completely),
             * than it is correct.
             */

            if(youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->hasGripper() &&
                        (rclcpp::Duration(node->now() - last_gripper_readings_time_).seconds() > (1 / youBotDriverGripperReadingsCycleFrequencyInHz))) {
                try {
                    youbot::YouBotGripperBar& gripperBar1 = youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1();
                    youbot::YouBotGripperBar& gripperBar2 = youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2();

                    gripperBar1.getData(gripperBar1Position);
                    gripperBar2.getData(gripperBar2Position);

                    armJointStateMessages[armIndex].name[youBotArmDoF + 0] = youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX];
                    double leftGipperFingerPosition = gripperBar1Position.barPosition.value();
                    armJointStateMessages[armIndex].position[youBotArmDoF + 0] = leftGipperFingerPosition;

                    double rightGipperFingerPosition = gripperBar2Position.barPosition.value();
                    armJointStateMessages[armIndex].name[youBotArmDoF + 1] = youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX];
                    armJointStateMessages[armIndex].position[youBotArmDoF + 1] = rightGipperFingerPosition;

                    last_gripper_readings_time_ = node->now();
                }
                catch (std::exception& e) {
                    std::string errorMessage = e.what();
                    RCLCPP_WARN(node->get_logger(), "Cannot read gripper values: %s", errorMessage.c_str());
                }
            }

            /*
            if (trajectoryActionServerEnable)
            {
                // updating joint states in trajectory action 
                youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction->jointStateCallback(armJointStateMessages[armIndex]);
            }
*/
        }
    }

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be received at the same time
    }catch (youbot::EtherCATConnectionException& e)
    {
        RCLCPP_WARN(node->get_logger(), "%s", e.what());
        youBotConfiguration.hasBase = false;
        youBotConfiguration.hasArms = false;
    }
    catch (std::exception& e)
    {
        RCLCPP_WARN_ONCE(node->get_logger(), "%s", e.what());
    }

}

void YouBotOODLWrapper::publishOODLSensorReadings()
{
      
    if (youBotConfiguration.hasBase)
    {
        youBotConfiguration.baseConfiguration.odometryBroadcaster->sendTransform(odometryTransform);
        youBotConfiguration.baseConfiguration.baseOdometryPublisher->publish(odometryMessage);
        youBotConfiguration.baseConfiguration.baseJointStatePublisher->publish(baseJointStateMessage);
    }

    if (youBotConfiguration.hasArms)
    {
        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++)
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher->publish(armJointStateMessages[armIndex]);
        }
    }


}

void YouBotOODLWrapper::switchOffBaseMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
	RCLCPP_INFO(node->get_logger(), "Switch off the base motors");
	if (youBotConfiguration.hasBase) { // in case stop has been invoked

        youbot::JointCurrentSetpoint currentStopMovement;
        currentStopMovement.current = 0.0 * ampere;
		try {
      youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(1).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(2).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(3).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(4).setData(currentStopMovement);
      youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			RCLCPP_WARN(node->get_logger(), "Cannot switch off the base motors: %s", errorMessage.c_str());
		}
	} else {
		RCLCPP_ERROR(node->get_logger(), "No base initialized!");
	}
  	areBaseMotorsSwitchedOn = false;
}

void YouBotOODLWrapper::switchOnBaseMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    RCLCPP_INFO(node->get_logger(), "Switch on the base motors");
    if (youBotConfiguration.hasBase)
    { // in case stop has been invoked
        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        longitudinalVelocity = 0.0 * meter_per_second;
        transversalVelocity = 0.0 * meter_per_second;
        angularVelocity = 0.0 * radian_per_second;

        try
        {
            youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            RCLCPP_WARN(node->get_logger(), "Cannot set base velocities: %s", errorMessage.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "No base initialized!");
    }
    areBaseMotorsSwitchedOn = true;
}

void YouBotOODLWrapper::switchOffArmMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response,
        int armIndex)
{
    RCLCPP_INFO(node->get_logger(), "Switch off the arm%i motors", armIndex+1);
    //TODO: is there a replacement for this in ros2?
    //assert(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) { // in case stop has been invoked

        youbot::JointCurrentSetpoint currentStopMovement;
        currentStopMovement.current = 0.0 * ampere;
        try{
            youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(1).setData(currentStopMovement);
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(2).setData(currentStopMovement);
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(3).setData(currentStopMovement);
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(4).setData(currentStopMovement);
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(5).setData(currentStopMovement);
            youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
        } catch (std::exception& e) {
            std::string errorMessage = e.what();
            RCLCPP_WARN(node->get_logger(), "Cannot switch off the arm motors: %s", errorMessage.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm%i not initialized!", armIndex+1);
    }
    areArmMotorsSwitchedOn = false;
}

void YouBotOODLWrapper::switchOnArmMotorsCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response,
        int armIndex)
{
    RCLCPP_INFO(node->get_logger(), "Switch on the arm%i motors", armIndex + 1);
    assert(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    {   
      try
        {
            std::vector<youbot::JointSensedAngle> sensedJointAngleVector;
            std::vector<youbot::JointAngleSetpoint> desiredJointAngleVector;
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getJointData(sensedJointAngleVector);
            youbot::JointAngleSetpoint desiredJointAngle;
            for(unsigned int i = 0; i < sensedJointAngleVector.size(); i++){
              desiredJointAngle = sensedJointAngleVector[i].angle;
              desiredJointAngleVector.push_back(desiredJointAngle);
            }        
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->setJointData(desiredJointAngleVector);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            RCLCPP_WARN(node->get_logger(), "Cannot switch on the arm motors: %s", errorMessage.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Arm%i not initialized!", armIndex + 1);
    }
    areArmMotorsSwitchedOn = true;
}

void YouBotOODLWrapper::calibrateArmCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response,
        int armIndex)
{
    RCLCPP_INFO(node->get_logger(), "Calibrate the arm%i", armIndex + 1);
    assert(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    {

        try
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateManipulator(true);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            RCLCPP_WARN(node->get_logger(), "Cannot calibrate the arm: %s", errorMessage.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Arm%i not initialized!", armIndex + 1);
    }
}

void YouBotOODLWrapper::reconnectCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
{

    this->stop();

    /* configuration */
    bool youBotHasBase;
    bool youBotHasArms;
    node->get_parameter("youBotHasBase", youBotHasBase);
    node->get_parameter("youBotHasArms", youBotHasArms);
    std::vector<std::string> armNames;

    // Retrieve all defined arm names from the launch file params
    int i = 1;
    std::stringstream armNameParam;
    armNameParam << "youBotArmName" << i; // youBotArmName1 is first checked param... then youBotArmName2, etc.
    while (node->has_parameter(armNameParam.str())) {
        std::string armName;
        node->get_parameter(armNameParam.str(), armName);
        armNames.push_back(armName);
        armNameParam.str("");
        armNameParam << "youBotArmName" <<  (++i);
    }

    assert((youBotHasBase == true) || (youBotHasArms == true)); // At least one should be true, otherwise nothing to be started.
    if (youBotHasBase == true)
    {
        this->initializeBase(this->youBotConfiguration.baseConfiguration.baseID);
    }

    if (youBotHasArms == true)
    {
        std::vector<std::string>::iterator armNameIter;
        for (armNameIter = armNames.begin(); armNameIter != armNames.end(); ++armNameIter) {
            this->initializeArm(*armNameIter);
        }
    }
}

void YouBotOODLWrapper::publishArmAndBaseDiagnostics(double publish_rate_in_secs) {
    // only publish every X seconds
    if ((node->now() - lastDiagnosticPublishTime).seconds() < publish_rate_in_secs)
        return;

    lastDiagnosticPublishTime = node->now();

    diagnosticArrayMessage.header.stamp = node->now();
    diagnosticArrayMessage.status.clear();

    // diagnostics message
    // base status
    diagnosticStatusMessage.name = diagnosticNameBase;
    if (youBotConfiguration.hasBase && areBaseMotorsSwitchedOn) {
        diagnosticStatusMessage.message = "base is present and switched on";
        diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (youBotConfiguration.hasBase && !areBaseMotorsSwitchedOn) {
        diagnosticStatusMessage.message = "base is switched off";
        diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
        diagnosticStatusMessage.message = "base is not enabled";
        diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);

    // arm status
    diagnosticStatusMessage.name = diagnosticNameArm;
    if (youBotConfiguration.hasArms && areArmMotorsSwitchedOn) {
        diagnosticStatusMessage.message = "arm is present and switched on";
        diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (youBotConfiguration.hasArms && !areArmMotorsSwitchedOn) {
        diagnosticStatusMessage.message = "arm is switched off";
        diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
        diagnosticStatusMessage.message = "arm is not enabled";
        diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);


    // EtherCAT status
    diagnosticStatusMessage.name = "platform_EtherCAT";
    try
    {
        if (youbot::EthercatMaster::getInstance().isEtherCATConnectionEstablished()) {
          diagnosticStatusMessage.message = "EtherCAT connnection is established";
          diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        }
        else {
          diagnosticStatusMessage.message = "EtherCAT connnection lost";
          diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        }
    } catch(std::exception &e) 
    {
        diagnosticStatusMessage.message = "EtherCAT connnection lost";
        diagnosticStatusMessage.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }    
    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);        

    // publish established messages
    diagnosticArrayPublisher->publish(diagnosticArrayMessage);
  }

} // namespace youBot

/* EOF */
