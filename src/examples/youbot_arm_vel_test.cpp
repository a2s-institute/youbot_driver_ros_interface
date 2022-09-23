/*
 * Copyright 2022 Bonn-Rhein-Sieg University
 *
 * Author: Tharun Sethuraman and Vamsi Kalagaturu
 *
 */

#include <iostream>
#include <assert.h>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "brics_actuator/msg/cartesian_wrench.hpp"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>

#include <iostream>
#include <assert.h>

#include "brics_actuator/msg/joint_positions.hpp"
#include "brics_actuator/msg/joint_velocities.hpp"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace std::this_thread; 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("youbot_arm_vel_test");
    auto armPositionsPublisher = node->create_publisher<brics_actuator::msg::JointVelocities> ("arm_1/arm_controller/velocity_command", 1);
    auto gripperPositionPublisher = node->create_publisher<brics_actuator::msg::JointVelocities> ("arm_1/gripper_controller/velocity_command", 1);

    double readValue;
    static const int numberOfArmJoints = 5;
    static const int numberOfGripperJoints = 2;

    rclcpp::WallRate loop_rate(50ms);

    while (rclcpp::ok()) {
        brics_actuator::msg::JointVelocities command;
        std::vector <brics_actuator::msg::JointValue> armJointPositions;
        std::vector <brics_actuator::msg::JointValue> gripperJointPositions;

        armJointPositions.resize(numberOfArmJoints); //TODO:change that
        gripperJointPositions.resize(numberOfGripperJoints);

        std::stringstream jointName;

        for (int i = 0; i < numberOfArmJoints; ++i)
        {
            std::cout << "Please type in velocity for joint " << i + 1 << std::endl;
            std::cin >> readValue;

            jointName.str("");
            jointName << "arm_joint_" << (i + 1);
            if(readValue<-0.2 || readValue>0.2)
            {
                std::cout << " Joint velocity exceeds the preferrable limit (-0.2 to 0.2)" << std::endl;
                std::cout << " Setting joint velocity to default value 0.0" << std::endl;
                readValue=0.0;
            }
            armJointPositions[i].joint_uri = jointName.str();
            armJointPositions[i].value = readValue;

            armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians_per_second);
            // std::cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << std::endl;

        }

        std::cout << "sending command ..." << std::endl;
        std::cout << "Joint velocities: " << armJointPositions[0].value << " " << armJointPositions[1].value << " " << armJointPositions[2].value << " " << armJointPositions[3].value << " " << armJointPositions[4].value << std::endl;

        try {
            command.velocities = armJointPositions;
            rclcpp::WallRate loop_rate_vel(50ms); 
            auto start_time = node -> get_clock() -> now().seconds();
            RCLCPP_INFO(node->get_logger(), "Publishing: [%f]", start_time);
            while (node -> get_clock() -> now().seconds() - start_time < 2)
            {
                armPositionsPublisher->publish(command);
                loop_rate_vel.sleep();
            }
            RCLCPP_INFO(node->get_logger(), "Publishing ended: [%f]", node -> get_clock() -> now().seconds());
            // publish 0 velocity to stop the arm
            for (int i = 0; i < numberOfArmJoints; ++i)
            {
                armJointPositions[i].value = 0.0;
            }
            command.velocities = armJointPositions;
            RCLCPP_INFO(node->get_logger(), "Publishing 0 velocities");
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
