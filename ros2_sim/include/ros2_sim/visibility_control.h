#ifndef ROS2_SIM__VISIBILITY_CONTROL_H_
#define ROS2_SIM__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROS2_SIM_EXPORT __attribute__ ((dllexport))
    #define ROS2_SIM_IMPORT __attribute__ ((dllimport))
  #else
    #define ROS2_SIM_EXPORT __declspec(dllexport)
    #define ROS2_SIM_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROS2_SIM_BUILDING_LIBRARY
    #define ROS2_SIM_PUBLIC ROS2_SIM_EXPORT
  #else
    #define ROS2_SIM_PUBLIC ROS2_SIM_IMPORT
  #endif
  #define ROS2_SIM_PUBLIC_TYPE ROS2_SIM_PUBLIC
  #define ROS2_SIM_LOCAL
#else
  #define ROS2_SIM_EXPORT __attribute__ ((visibility("default")))
  #define ROS2_SIM_IMPORT
  #if __GNUC__ >= 4
    #define ROS2_SIM_PUBLIC __attribute__ ((visibility("default")))
    #define ROS2_SIM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROS2_SIM_PUBLIC
    #define ROS2_SIM_LOCAL
  #endif
  #define ROS2_SIM_PUBLIC_TYPE
#endif
#endif  // ROS2_SIM__VISIBILITY_CONTROL_H_
// Generated 04-Jul-2025 15:34:10
 