#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_builtin_interfaces_Time and builtin_interfaces::msg::Time

void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  msgPtr.nanosec =  busPtr->nanosec;
  msgPtr.sec =  busPtr->sec;
}

void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  busPtr->nanosec =  msgPtr.nanosec;
  busPtr->sec =  msgPtr.sec;
}


// Conversions between SL_Bus_sensor_msgs_BatteryState and sensor_msgs::msg::BatteryState

void convertFromBus(sensor_msgs::msg::BatteryState& msgPtr, SL_Bus_sensor_msgs_BatteryState const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/BatteryState");

  msgPtr.capacity =  busPtr->capacity;
  convertFromBusVariablePrimitiveArray(msgPtr.cell_temperature, busPtr->cell_temperature, busPtr->cell_temperature_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr.cell_voltage, busPtr->cell_voltage, busPtr->cell_voltage_SL_Info);
  msgPtr.charge =  busPtr->charge;
  msgPtr.current =  busPtr->current;
  msgPtr.design_capacity =  busPtr->design_capacity;
  convertFromBus(msgPtr.header, &busPtr->header);
  convertFromBusVariablePrimitiveArray(msgPtr.location, busPtr->location, busPtr->location_SL_Info);
  msgPtr.percentage =  busPtr->percentage;
  msgPtr.power_supply_health =  busPtr->power_supply_health;
  msgPtr.power_supply_status =  busPtr->power_supply_status;
  msgPtr.power_supply_technology =  busPtr->power_supply_technology;
  msgPtr.present =  busPtr->present;
  convertFromBusVariablePrimitiveArray(msgPtr.serial_number, busPtr->serial_number, busPtr->serial_number_SL_Info);
  msgPtr.temperature =  busPtr->temperature;
  msgPtr.voltage =  busPtr->voltage;
}

void convertToBus(SL_Bus_sensor_msgs_BatteryState* busPtr, const sensor_msgs::msg::BatteryState& msgPtr)
{
  const std::string rosMessageType("sensor_msgs/BatteryState");

  busPtr->capacity =  msgPtr.capacity;
  convertToBusVariablePrimitiveArray(busPtr->cell_temperature, busPtr->cell_temperature_SL_Info, msgPtr.cell_temperature, slros::EnabledWarning(rosMessageType, "cell_temperature"));
  convertToBusVariablePrimitiveArray(busPtr->cell_voltage, busPtr->cell_voltage_SL_Info, msgPtr.cell_voltage, slros::EnabledWarning(rosMessageType, "cell_voltage"));
  busPtr->charge =  msgPtr.charge;
  busPtr->current =  msgPtr.current;
  busPtr->design_capacity =  msgPtr.design_capacity;
  convertToBus(&busPtr->header, msgPtr.header);
  convertToBusVariablePrimitiveArray(busPtr->location, busPtr->location_SL_Info, msgPtr.location, slros::EnabledWarning(rosMessageType, "location"));
  busPtr->percentage =  msgPtr.percentage;
  busPtr->power_supply_health =  msgPtr.power_supply_health;
  busPtr->power_supply_status =  msgPtr.power_supply_status;
  busPtr->power_supply_technology =  msgPtr.power_supply_technology;
  busPtr->present =  msgPtr.present;
  convertToBusVariablePrimitiveArray(busPtr->serial_number, busPtr->serial_number_SL_Info, msgPtr.serial_number, slros::EnabledWarning(rosMessageType, "serial_number"));
  busPtr->temperature =  msgPtr.temperature;
  busPtr->voltage =  msgPtr.voltage;
}


// Conversions between SL_Bus_std_msgs_Float32 and std_msgs::msg::Float32

void convertFromBus(std_msgs::msg::Float32& msgPtr, SL_Bus_std_msgs_Float32 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float32");

  msgPtr.data =  busPtr->data;
}

void convertToBus(SL_Bus_std_msgs_Float32* busPtr, const std_msgs::msg::Float32& msgPtr)
{
  const std::string rosMessageType("std_msgs/Float32");

  busPtr->data =  msgPtr.data;
}


// Conversions between SL_Bus_std_msgs_Header and std_msgs::msg::Header

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr.frame_id, busPtr->frame_id, busPtr->frame_id_SL_Info);
  convertFromBus(msgPtr.stamp, &busPtr->stamp);
}

void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->frame_id, busPtr->frame_id_SL_Info, msgPtr.frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  convertToBus(&busPtr->stamp, msgPtr.stamp);
}

