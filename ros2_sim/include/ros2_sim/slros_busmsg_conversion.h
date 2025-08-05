#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include "ros2_sim_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr);
void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr);

void convertFromBus(sensor_msgs::msg::BatteryState& msgPtr, SL_Bus_sensor_msgs_BatteryState const* busPtr);
void convertToBus(SL_Bus_sensor_msgs_BatteryState* busPtr, const sensor_msgs::msg::BatteryState& msgPtr);

void convertFromBus(std_msgs::msg::Float32& msgPtr, SL_Bus_std_msgs_Float32 const* busPtr);
void convertToBus(SL_Bus_std_msgs_Float32* busPtr, const std_msgs::msg::Float32& msgPtr);

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr);


#endif
