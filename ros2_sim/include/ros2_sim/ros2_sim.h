//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ros2_sim.h
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
#ifndef ros2_sim_h_
#define ros2_sim_h_
#include "rtwtypes.h"
#include "slros2_initialize.h"
#include "ros2_sim_types.h"
#include <math.h>
#include <stddef.h>

// Block signals for system '<Root>/SOC Estimator (Kalman Filter)'
struct B_CoreSubsys_ros2_sim_T {
  real32_T Probe[2];                   // '<S13>/Probe'
};

// Block states (default storage) for system '<Root>/SOC Estimator (Kalman Filter)' 
struct DW_CoreSubsys_ros2_sim_T {
  real32_T Delay_DSTATE[3];            // '<S12>/Delay'
  real32_T UnitDelayP_DSTATE[9];       // '<S10>/Unit Delay - P'
  boolean_T icLoad;                    // '<S12>/Delay'
};

// Block signals (default storage)
struct B_ros2_sim_T {
  SL_Bus_sensor_msgs_BatteryState In1; // '<S23>/In1'
  SL_Bus_sensor_msgs_BatteryState BusAssignment;// '<Root>/Bus Assignment'
  real32_T Assignment1[9];             // '<S13>/Assignment1'
  real32_T Sum1_n[9];                  // '<S14>/Sum1'
  real32_T fv[9];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_m;
  sJ4ih70VmKcvCeguWN0mNVF deadline_c;
  real32_T Product3_k[3];              // '<S11>/Product3'
  real32_T TmpSignalConversionAtProduc[3];
  real32_T Sum_g5[3];                  // '<S11>/Sum'
  real32_T fv1[3];
  real_T WhiteNoise;                   // '<S8>/White Noise'
  real_T rtb_FromWorkspace_k;
  real_T sr;
  real_T si;
  real32_T u;                          // '<Root>/Sum1'
  SL_Bus_std_msgs_Float32 BusAssignment1;// '<Root>/Bus Assignment1'
  B_CoreSubsys_ros2_sim_T CoreSubsys[1];
                                      // '<Root>/SOC Estimator (Kalman Filter)'
};

// Block states (default storage) for system '<Root>'
struct DW_ros2_sim_T {
  ros_slros2_internal_block_Pub_T obj; // '<S5>/SinkBlock'
  ros_slros2_internal_block_Pub_T obj_j;// '<S4>/SinkBlock'
  ros_slros2_internal_block_Sub_T obj_l;// '<S7>/SourceBlock'
  real_T NextOutput;                   // '<S3>/White Noise'
  real_T NextOutput_g;                 // '<S8>/White Noise'
  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace1_PWORK;              // '<Root>/From Workspace1'

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK;               // '<Root>/From Workspace'

  uint32_T RandSeed;                   // '<S3>/White Noise'
  uint32_T RandSeed_b;                 // '<S8>/White Noise'
  struct {
    int_T PrevIndex;
  } FromWorkspace1_IWORK;              // '<Root>/From Workspace1'

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK;               // '<Root>/From Workspace'

  DW_CoreSubsys_ros2_sim_T CoreSubsys[1];
                                      // '<Root>/SOC Estimator (Kalman Filter)'
};

// Invariant block signals for system '<Root>/SOC Estimator (Kalman Filter)'
struct ConstB_CoreSubsys_ros2_sim_T {
  real32_T Conversion;                 // '<S16>/Conversion'
  real32_T Conversion_b[9];            // '<S17>/Conversion'
  real32_T Conversion_d;               // '<S18>/Conversion'
  real32_T Conversion_k[9];            // '<S19>/Conversion'
  real32_T Conversion_g;               // '<S20>/Conversion'
  real32_T Conversion_m;               // '<S21>/Conversion'
};

// Invariant block signals (default storage)
struct ConstB_ros2_sim_T {
  ConstB_CoreSubsys_ros2_sim_T CoreSubsys;
                                      // '<Root>/SOC Estimator (Kalman Filter)'
};

// Constant parameters (default storage)
struct ConstP_ros2_sim_T {
  // Expression: Q_2RC
  //  Referenced by: '<S14>/Constant'

  real_T Constant_Value_b[9];

  // Computed Parameter: uDLookupTableOCV1_tableData
  //  Referenced by: '<S13>/2-D Lookup Table OCV1'

  real32_T uDLookupTableOCV1_tableData[66];

  // Expression: SOC_vec
  //  Referenced by: '<S13>/2-D Lookup Table OCV1'

  real32_T uDLookupTableOCV1_bp01Data[33];

  // Expression: T_vec
  //  Referenced by: '<S13>/2-D Lookup Table OCV1'

  real32_T uDLookupTableOCV1_bp02Data[2];

  // Expression: R2_mat
  //  Referenced by: '<S13>/2-D Lookup Table R2'

  real32_T uDLookupTableR2_tableData[66];

  // Expression: SOC_vec
  //  Referenced by: '<S13>/2-D Lookup Table R2'

  real32_T uDLookupTableR2_bp01Data[33];

  // Expression: T_vec
  //  Referenced by: '<S13>/2-D Lookup Table R2'

  real32_T uDLookupTableR2_bp02Data[2];

  // Computed Parameter: uDLookupTableC2_tableData
  //  Referenced by: '<S13>/2-D Lookup Table C2'

  real32_T uDLookupTableC2_tableData[66];

  // Expression: SOC_vec
  //  Referenced by: '<S13>/2-D Lookup Table C2'

  real32_T uDLookupTableC2_bp01Data[33];

  // Expression: T_vec
  //  Referenced by: '<S13>/2-D Lookup Table C2'

  real32_T uDLookupTableC2_bp02Data[2];

  // Expression: R1_mat
  //  Referenced by: '<S13>/2-D Lookup Table R1'

  real32_T uDLookupTableR1_tableData[66];

  // Expression: SOC_vec
  //  Referenced by: '<S13>/2-D Lookup Table R1'

  real32_T uDLookupTableR1_bp01Data[33];

  // Expression: T_vec
  //  Referenced by: '<S13>/2-D Lookup Table R1'

  real32_T uDLookupTableR1_bp02Data[2];

  // Computed Parameter: uDLookupTableC1_tableData
  //  Referenced by: '<S13>/2-D Lookup Table C1'

  real32_T uDLookupTableC1_tableData[66];

  // Expression: SOC_vec
  //  Referenced by: '<S13>/2-D Lookup Table C1'

  real32_T uDLookupTableC1_bp01Data[33];

  // Expression: T_vec
  //  Referenced by: '<S13>/2-D Lookup Table C1'

  real32_T uDLookupTableC1_bp02Data[2];

  // Expression: P0_2RC
  //  Referenced by: '<S10>/Unit Delay - P'

  real32_T UnitDelayP_InitialCondition[9];

  // Expression: V0_mat
  //  Referenced by: '<S13>/2-D Lookup Table OCV'

  real32_T uDLookupTableOCV_tableData[66];

  // Expression: SOC_vec
  //  Referenced by: '<S13>/2-D Lookup Table OCV'

  real32_T uDLookupTableOCV_bp01Data[33];

  // Expression: T_vec
  //  Referenced by: '<S13>/2-D Lookup Table OCV'

  real32_T uDLookupTableOCV_bp02Data[2];

  // Expression: R0_mat
  //  Referenced by: '<S13>/2-D Lookup Table R0'

  real32_T uDLookupTableR0_tableData[66];

  // Expression: SOC_vec
  //  Referenced by: '<S13>/2-D Lookup Table R0'

  real32_T uDLookupTableR0_bp01Data[33];

  // Expression: T_vec
  //  Referenced by: '<S13>/2-D Lookup Table R0'

  real32_T uDLookupTableR0_bp02Data[2];

  // Pooled Parameter (Expression: )
  //  Referenced by:
  //    '<S13>/2-D Lookup Table C1'
  //    '<S13>/2-D Lookup Table C2'
  //    '<S13>/2-D Lookup Table OCV'
  //    '<S13>/2-D Lookup Table OCV1'
  //    '<S13>/2-D Lookup Table R0'
  //    '<S13>/2-D Lookup Table R1'
  //    '<S13>/2-D Lookup Table R2'

  uint32_T pooled7[2];
};

// Real-time Model Data Structure
struct tag_RTM_ros2_sim_T {
  const char_T * volatile errorStatus;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
  } Timing;

  const char_T* getErrorStatus() const;
  void setErrorStatus(const char_T* const volatile aErrorStatus);
};

extern const ConstB_ros2_sim_T ros2_sim_ConstB;// constant block i/o

// Constant parameters (default storage)
extern const ConstP_ros2_sim_T ros2_sim_ConstP;

// Class declaration for model ros2_sim
class ros2_sim
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_ros2_sim_T * getRTM();

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  ros2_sim();

  // Destructor
  ~ros2_sim();

  // private data and function members
 private:
  // Block signals
  B_ros2_sim_T ros2_sim_B;

  // Block states
  DW_ros2_sim_T ros2_sim_DW;

  // private member function(s) for subsystem '<Root>'
  real_T ros2_rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u);
  void ros2_sim_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T *obj);
  void ros2_sim_Publisher_setupImpl_d(const ros_slros2_internal_block_Pub_T *obj);
  void ros2_sim_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T *obj);

  // Real-Time Model
  RT_MODEL_ros2_sim_T ros2_sim_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Display' : Unused code path elimination
//  Block '<S15>/Data Type Duplicate' : Unused code path elimination
//  Block '<S16>/Data Type Duplicate' : Unused code path elimination
//  Block '<S17>/Data Type Duplicate' : Unused code path elimination
//  Block '<S18>/Data Type Duplicate' : Unused code path elimination
//  Block '<S19>/Data Type Duplicate' : Unused code path elimination
//  Block '<S20>/Data Type Duplicate' : Unused code path elimination
//  Block '<S21>/Data Type Duplicate' : Unused code path elimination
//  Block '<S22>/Data Type Duplicate' : Unused code path elimination
//  Block '<S10>/Rate Transition' : Eliminated since input and output rates are identical
//  Block '<S10>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S10>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S10>/Rate Transition3' : Eliminated since input and output rates are identical


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ros2_sim'
//  '<S1>'   : 'ros2_sim/Blank Message'
//  '<S2>'   : 'ros2_sim/Blank Message1'
//  '<S3>'   : 'ros2_sim/Current sensor noise1'
//  '<S4>'   : 'ros2_sim/Publish1'
//  '<S5>'   : 'ros2_sim/Publish2'
//  '<S6>'   : 'ros2_sim/SOC Estimator (Kalman Filter)'
//  '<S7>'   : 'ros2_sim/Subscribe'
//  '<S8>'   : 'ros2_sim/Voltage sensor noise'
//  '<S9>'   : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter'
//  '<S10>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC'
//  '<S11>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Correction'
//  '<S12>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Delay - X'
//  '<S13>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Jacobian'
//  '<S14>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Prediction'
//  '<S15>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Correction/Data Type Conversion Inherited'
//  '<S16>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Correction/Data Type Conversion Inherited1'
//  '<S17>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Correction/Data Type Conversion Inherited2'
//  '<S18>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Delay - X/Data Type Conversion Inherited'
//  '<S19>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Jacobian/Data Type Conversion Inherited'
//  '<S20>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Jacobian/Data Type Conversion Inherited1'
//  '<S21>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Jacobian/Data Type Conversion Inherited2'
//  '<S22>'  : 'ros2_sim/SOC Estimator (Kalman Filter)/Kalman Filter/EKF 2RC/Prediction/Data Type Conversion Inherited'
//  '<S23>'  : 'ros2_sim/Subscribe/Enabled Subsystem'

#endif                                 // ros2_sim_h_

//
// File trailer for generated code.
//
// [EOF]
//
