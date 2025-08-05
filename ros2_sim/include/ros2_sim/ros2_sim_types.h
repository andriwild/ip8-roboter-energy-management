//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ros2_sim_types.h
//
// Code generated for Simulink model 'ros2_sim'.
//
// Model version                  : 1.3
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Fri Jul  4 15:34:02 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef ros2_sim_types_h_
#define ros2_sim_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float32_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float32_

// MsgType=std_msgs/Float32
struct SL_Bus_std_msgs_Float32
{
  real32_T data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_

// MsgType=builtin_interfaces/Time
struct SL_Bus_builtin_interfaces_Time
{
  int32_T sec;
  uint32_T nanosec;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_

// MsgType=std_msgs/Header
struct SL_Bus_std_msgs_Header
{
  // MsgType=builtin_interfaces/Time
  SL_Bus_builtin_interfaces_Time stamp;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=frame_id_SL_Info:TruncateAction=warn 
  uint8_T frame_id[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=frame_id
  SL_Bus_ROSVariableLengthArrayInfo frame_id_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_BatteryState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_BatteryState_

// MsgType=sensor_msgs/BatteryState
struct SL_Bus_sensor_msgs_BatteryState
{
  // MsgType=std_msgs/Header
  SL_Bus_std_msgs_Header header;
  real32_T voltage;
  real32_T temperature;
  real32_T current;
  real32_T charge;
  real32_T capacity;
  real32_T design_capacity;
  real32_T percentage;
  uint8_T power_supply_status;
  uint8_T power_supply_health;
  uint8_T power_supply_technology;
  boolean_T present;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=cell_voltage_SL_Info:TruncateAction=warn 
  real32_T cell_voltage[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=cell_voltage
  SL_Bus_ROSVariableLengthArrayInfo cell_voltage_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=cell_temperature_SL_Info:TruncateAction=warn 
  real32_T cell_temperature[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=cell_temperature
  SL_Bus_ROSVariableLengthArrayInfo cell_temperature_SL_Info;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=location_SL_Info:TruncateAction=warn 
  uint8_T location[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=location
  SL_Bus_ROSVariableLengthArrayInfo location_SL_Info;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=serial_number_SL_Info:TruncateAction=warn 
  uint8_T serial_number[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=serial_number
  SL_Bus_ROSVariableLengthArrayInfo serial_number_SL_Info;
};

#endif

// Custom Type definition for MATLABSystem: '<S7>/SourceBlock'
#include "rmw/qos_profiles.h"
#ifndef struct_sJ4ih70VmKcvCeguWN0mNVF
#define struct_sJ4ih70VmKcvCeguWN0mNVF

struct sJ4ih70VmKcvCeguWN0mNVF
{
  real_T sec;
  real_T nsec;
};

#endif                                 // struct_sJ4ih70VmKcvCeguWN0mNVF

#ifndef struct_ros_slros2_internal_block_Pub_T
#define struct_ros_slros2_internal_block_Pub_T

struct ros_slros2_internal_block_Pub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                                // struct_ros_slros2_internal_block_Pub_T

#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                                // struct_ros_slros2_internal_block_Sub_T

// Forward declaration for rtModel
typedef struct tag_RTM_ros2_sim_T RT_MODEL_ros2_sim_T;

#endif                                 // ros2_sim_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
