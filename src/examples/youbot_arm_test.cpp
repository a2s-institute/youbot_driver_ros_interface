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

#include <iostream>
#include <assert.h>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "brics_actuator/msg/cartesian_wrench.hpp"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "brics_actuator/msg/joint_positions.hpp"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("youbot_arm_test");
    auto armPositionsPublisher = node->create_publisher<brics_actuator::msg::JointPositions> ("arm_1/arm_controller/position_command", 1);
    auto gripperPositionPublisher = node->create_publisher<brics_actuator::msg::JointPositions> ("arm_1/gripper_controller/position_command", 1);

    double readValue;
    static const int numberOfArmJoints = 5;
    static const int numberOfGripperJoints = 2;

    rclcpp::WallRate loop_rate(50ms);

    while (rclcpp::ok()) {
        brics_actuator::msg::JointPositions command;
        std::vector <brics_actuator::msg::JointValue> armJointPositions;
        std::vector <brics_actuator::msg::JointValue> gripperJointPositions;

        armJointPositions.resize(numberOfArmJoints); //TODO:change that
        gripperJointPositions.resize(numberOfGripperJoints);

        std::stringstream jointName;


        // ::io::base_unit_info <boost::units::si::angular_velocity>).name();
        for (int i = 0; i < numberOfArmJoints; ++i)
        {
            std::cout << "Please type in value for joint " << i + 1 << std::endl;
            std::cin >> readValue;

            jointName.str("");
            jointName << "arm_joint_" << (i + 1);

            armJointPositions[i].joint_uri = jointName.str();
            armJointPositions[i].value = readValue;

            armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
            std::cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << std::endl;

        }

        //		std::cout << "Please type in value for a left jaw of the gripper " << std::endl;
        //		std::cin >> readValue;
        //		gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
        //		gripperJointPositions[0].value = readValue;
        //		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

        //		std::cout << "Please type in value for a right jaw of the gripper " << std::endl;
        //		std::cin >> readValue;
        //		gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
        //		gripperJointPositions[1].value = readValue;
        //		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

        std::cout << "sending command ..." << std::endl;


        //		command.positions = gripperJointPositions;
        //		gripperPositionPublisher->publish(command);

        try {
            command.positions = armJointPositions;
            armPositionsPublisher->publish(command);

            rclcpp::spin_some(node);
        } catch (const rclcpp::exceptions::RCLError & e) {
          RCLCPP_ERROR(
            node->get_logger(),
            "unexpectedly failed with %s",
            e.what());
        }
        std::cout << "--------------------" << std::endl;
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

/* EOF */
