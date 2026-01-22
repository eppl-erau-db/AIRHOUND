// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x11, 0xf4, 0x32, 0xe7, 0x7e, 0x73, 0x96, 0xc6,
      0xcc, 0xf0, 0xc6, 0x4d, 0x65, 0x56, 0x80, 0x9b,
      0x1e, 0x9a, 0xec, 0x4c, 0xf4, 0x81, 0x2b, 0xcb,
      0x56, 0x99, 0x13, 0x79, 0x2c, 0xe9, 0x96, 0x5b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorStatus__TYPE_NAME[] = "px4_msgs/msg/EstimatorStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__output_tracking_error[] = "output_tracking_error";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__gps_check_fail_flags[] = "gps_check_fail_flags";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__control_mode_flags[] = "control_mode_flags";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__filter_fault_flags[] = "filter_fault_flags";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pos_horiz_accuracy[] = "pos_horiz_accuracy";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pos_vert_accuracy[] = "pos_vert_accuracy";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__hdg_test_ratio[] = "hdg_test_ratio";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__vel_test_ratio[] = "vel_test_ratio";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pos_test_ratio[] = "pos_test_ratio";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__hgt_test_ratio[] = "hgt_test_ratio";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__tas_test_ratio[] = "tas_test_ratio";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__hagl_test_ratio[] = "hagl_test_ratio";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__beta_test_ratio[] = "beta_test_ratio";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__solution_status_flags[] = "solution_status_flags";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_vel_ne[] = "reset_count_vel_ne";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_vel_d[] = "reset_count_vel_d";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_pos_ne[] = "reset_count_pos_ne";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_pod_d[] = "reset_count_pod_d";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_quat[] = "reset_count_quat";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__time_slip[] = "time_slip";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_heading[] = "pre_flt_fail_innov_heading";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_height[] = "pre_flt_fail_innov_height";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_pos_horiz[] = "pre_flt_fail_innov_pos_horiz";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_vel_horiz[] = "pre_flt_fail_innov_vel_horiz";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_vel_vert[] = "pre_flt_fail_innov_vel_vert";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_mag_field_disturbed[] = "pre_flt_fail_mag_field_disturbed";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__accel_device_id[] = "accel_device_id";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__gyro_device_id[] = "gyro_device_id";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__baro_device_id[] = "baro_device_id";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_device_id[] = "mag_device_id";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__health_flags[] = "health_flags";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__timeout_flags[] = "timeout_flags";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_inclination_deg[] = "mag_inclination_deg";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_inclination_ref_deg[] = "mag_inclination_ref_deg";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_strength_gs[] = "mag_strength_gs";
static char px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_strength_ref_gs[] = "mag_strength_ref_gs";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorStatus__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__output_tracking_error, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__gps_check_fail_flags, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__control_mode_flags, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__filter_fault_flags, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pos_horiz_accuracy, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pos_vert_accuracy, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__hdg_test_ratio, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__vel_test_ratio, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pos_test_ratio, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__hgt_test_ratio, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__tas_test_ratio, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__hagl_test_ratio, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__beta_test_ratio, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__solution_status_flags, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_vel_ne, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_vel_d, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_pos_ne, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_pod_d, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__reset_count_quat, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__time_slip, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_heading, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_height, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_pos_horiz, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_vel_horiz, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_innov_vel_vert, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__pre_flt_fail_mag_field_disturbed, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__accel_device_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__gyro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__baro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_device_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__health_flags, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__timeout_flags, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_inclination_deg, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_inclination_ref_deg, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_strength_gs, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatus__FIELD_NAME__mag_strength_ref_gs, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__EstimatorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorStatus__TYPE_NAME, 28, 28},
      {px4_msgs__msg__EstimatorStatus__FIELDS, 38, 38},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint64 timestamp_sample         # the timestamp of the raw data (microseconds)\n"
  "\n"
  "float32[3] output_tracking_error # return a vector containing the output predictor angular, velocity and position tracking error magnitudes (rad), (m/s), (m)\n"
  "\n"
  "uint16 gps_check_fail_flags     # Bitmask to indicate status of GPS checks - see definition below\n"
  "# bits are true when corresponding test has failed\n"
  "uint8 GPS_CHECK_FAIL_GPS_FIX = 0\\t\\t# 0 : insufficient fix type (no 3D solution)\n"
  "uint8 GPS_CHECK_FAIL_MIN_SAT_COUNT = 1\\t\\t# 1 : minimum required sat count fail\n"
  "uint8 GPS_CHECK_FAIL_MAX_PDOP = 2\\t\\t# 2 : maximum allowed PDOP fail\n"
  "uint8 GPS_CHECK_FAIL_MAX_HORZ_ERR = 3\\t\\t# 3 : maximum allowed horizontal position error fail\n"
  "uint8 GPS_CHECK_FAIL_MAX_VERT_ERR = 4\\t\\t# 4 : maximum allowed vertical position error fail\n"
  "uint8 GPS_CHECK_FAIL_MAX_SPD_ERR = 5\\t\\t# 5 : maximum allowed speed error fail\n"
  "uint8 GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 6\\t\\t# 6 : maximum allowed horizontal position drift fail - requires stationary vehicle\n"
  "uint8 GPS_CHECK_FAIL_MAX_VERT_DRIFT = 7\\t\\t# 7 : maximum allowed vertical position drift fail - requires stationary vehicle\n"
  "uint8 GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 8\\t# 8 : maximum allowed horizontal speed fail - requires stationary vehicle\n"
  "uint8 GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 9\\t# 9 : maximum allowed vertical velocity discrepancy fail\n"
  "uint8 GPS_CHECK_FAIL_SPOOFED = 10\\t\\t# 10 : GPS signal is spoofed\n"
  "\n"
  "uint64 control_mode_flags\\t# Bitmask to indicate EKF logic state\n"
  "uint8 CS_TILT_ALIGN = 0\\t\\t# 0 - true if the filter tilt alignment is complete\n"
  "uint8 CS_YAW_ALIGN = 1\\t\\t# 1 - true if the filter yaw alignment is complete\n"
  "uint8 CS_GNSS_POS = 2\\t\\t# 2 - true if GNSS position measurements are being fused\n"
  "uint8 CS_OPT_FLOW = 3\\t\\t# 3 - true if optical flow measurements are being fused\n"
  "uint8 CS_MAG_HDG = 4\\t\\t# 4 - true if a simple magnetic yaw heading is being fused\n"
  "uint8 CS_MAG_3D = 5\\t\\t# 5 - true if 3-axis magnetometer measurement are being fused\n"
  "uint8 CS_MAG_DEC = 6\\t\\t# 6 - true if synthetic magnetic declination measurements are being fused\n"
  "uint8 CS_IN_AIR = 7\\t\\t# 7 - true when thought to be airborne\n"
  "uint8 CS_WIND = 8\\t\\t# 8 - true when wind velocity is being estimated\n"
  "uint8 CS_BARO_HGT = 9\\t\\t# 9 - true when baro data is being fused\n"
  "uint8 CS_RNG_HGT = 10\\t\\t# 10 - true when range finder data is being fused for height aiding\n"
  "uint8 CS_GPS_HGT = 11\\t\\t# 11 - true when GPS altitude is being fused\n"
  "uint8 CS_EV_POS = 12\\t\\t# 12 - true when local position data from external vision is being fused\n"
  "uint8 CS_EV_YAW = 13\\t\\t# 13 - true when yaw data from external vision measurements is being fused\n"
  "uint8 CS_EV_HGT = 14\\t\\t# 14 - true when height data from external vision measurements is being fused\n"
  "uint8 CS_BETA = 15\\t\\t# 15 - true when synthetic sideslip measurements are being fused\n"
  "uint8 CS_MAG_FIELD = 16\\t\\t# 16 - true when only the magnetic field states are updated by the magnetometer\n"
  "uint8 CS_FIXED_WING = 17\\t# 17 - true when thought to be operating as a fixed wing vehicle with constrained sideslip\n"
  "uint8 CS_MAG_FAULT = 18\\t\\t# 18 - true when the magnetometer has been declared faulty and is no longer being used\n"
  "uint8 CS_ASPD = 19\\t\\t# 19 - true when airspeed measurements are being fused\n"
  "uint8 CS_GND_EFFECT = 20\\t# 20 - true when when protection from ground effect induced static pressure rise is active\n"
  "uint8 CS_RNG_STUCK = 21\\t\\t# 21 - true when a stuck range finder sensor has been detected\n"
  "uint8 CS_GPS_YAW = 22\\t\\t# 22 - true when yaw (not ground course) data from a GPS receiver is being fused\n"
  "uint8 CS_MAG_ALIGNED = 23\\t# 23 - true when the in-flight mag field alignment has been completed\n"
  "uint8 CS_EV_VEL = 24\\t\\t# 24 - true when local frame velocity data fusion from external vision measurements is intended\n"
  "uint8 CS_SYNTHETIC_MAG_Z = 25\\t# 25 - true when we are using a synthesized measurement for the magnetometer Z component\n"
  "uint8 CS_VEHICLE_AT_REST = 26\\t# 26 - true when the vehicle is at rest\n"
  "uint8 CS_GPS_YAW_FAULT = 27\\t# 27 - true when the GNSS heading has been declared faulty and is no longer being used\n"
  "uint8 CS_RNG_FAULT = 28\\t\\t# 28 - true when the range finder has been declared faulty and is no longer being used\n"
  "uint8 CS_GNSS_VEL = 44\\t\\t# 44 - true if GNSS velocity measurement fusion is intended\n"
  "uint8 CS_GNSS_FAULT = 45        # 45 - true if GNSS measurements have been declared faulty and are no longer used\n"
  "uint8 CS_YAW_MANUAL = 46        # 46 - true if yaw has been set manually\n"
  "\n"
  "uint32 filter_fault_flags\\t# Bitmask to indicate EKF internal faults\n"
  "# 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error\n"
  "# 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error\n"
  "# 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error\n"
  "# 3 - true if the fusion of the magnetic heading has encountered a numerical error\n"
  "# 4 - true if the fusion of the magnetic declination has encountered a numerical error\n"
  "# 5 - true if fusion of the airspeed has encountered a numerical error\n"
  "# 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error\n"
  "# 7 - true if fusion of the optical flow X axis has encountered a numerical error\n"
  "# 8 - true if fusion of the optical flow Y axis has encountered a numerical error\n"
  "# 9 - true if fusion of the North velocity has encountered a numerical error\n"
  "# 10 - true if fusion of the East velocity has encountered a numerical error\n"
  "# 11 - true if fusion of the Down velocity has encountered a numerical error\n"
  "# 12 - true if fusion of the North position has encountered a numerical error\n"
  "# 13 - true if fusion of the East position has encountered a numerical error\n"
  "# 14 - true if fusion of the Down position has encountered a numerical error\n"
  "# 15 - true if bad delta velocity bias estimates have been detected\n"
  "# 16 - true if bad vertical accelerometer data has been detected\n"
  "# 17 - true if delta velocity data contains clipping (asymmetric railing)\n"
  "\n"
  "float32 pos_horiz_accuracy # 1-Sigma estimated horizontal position accuracy relative to the estimators origin (m)\n"
  "float32 pos_vert_accuracy # 1-Sigma estimated vertical position accuracy relative to the estimators origin (m)\n"
  "\n"
  "float32 hdg_test_ratio # low-pass filtered ratio of the largest heading innovation component to the innovation test limit\n"
  "float32 vel_test_ratio # low-pass filtered ratio of the largest velocity innovation component to the innovation test limit\n"
  "float32 pos_test_ratio # low-pass filtered ratio of the largest horizontal position innovation component to the innovation test limit\n"
  "float32 hgt_test_ratio # low-pass filtered ratio of the vertical position innovation to the innovation test limit\n"
  "float32 tas_test_ratio # low-pass filtered ratio of the true airspeed innovation to the innovation test limit\n"
  "float32 hagl_test_ratio # low-pass filtered ratio of the height above ground innovation to the innovation test limit\n"
  "float32 beta_test_ratio # low-pass filtered ratio of the synthetic sideslip innovation to the innovation test limit\n"
  "\n"
  "uint16 solution_status_flags # Bitmask indicating which filter kinematic state outputs are valid for flight control use.\n"
  "# 0 - True if the attitude estimate is good\n"
  "# 1 - True if the horizontal velocity estimate is good\n"
  "# 2 - True if the vertical velocity estimate is good\n"
  "# 3 - True if the horizontal position (relative) estimate is good\n"
  "# 4 - True if the horizontal position (absolute) estimate is good\n"
  "# 5 - True if the vertical position (absolute) estimate is good\n"
  "# 6 - True if the vertical position (above ground) estimate is good\n"
  "# 7 - True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)\n"
  "# 8 - True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate\n"
  "# 9 - True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate\n"
  "# 10 - True if the EKF has detected a GPS glitch\n"
  "# 11 - True if the EKF has detected bad accelerometer data\n"
  "\n"
  "uint8 reset_count_vel_ne # number of horizontal position reset events (allow to wrap if count exceeds 255)\n"
  "uint8 reset_count_vel_d  # number of vertical velocity reset events (allow to wrap if count exceeds 255)\n"
  "uint8 reset_count_pos_ne # number of horizontal position reset events (allow to wrap if count exceeds 255)\n"
  "uint8 reset_count_pod_d  # number of vertical position reset events (allow to wrap if count exceeds 255)\n"
  "uint8 reset_count_quat   # number of quaternion reset events (allow to wrap if count exceeds 255)\n"
  "\n"
  "float32 time_slip # cumulative amount of time in seconds that the EKF inertial calculation has slipped relative to system time\n"
  "\n"
  "bool pre_flt_fail_innov_heading\n"
  "bool pre_flt_fail_innov_height\n"
  "bool pre_flt_fail_innov_pos_horiz\n"
  "bool pre_flt_fail_innov_vel_horiz\n"
  "bool pre_flt_fail_innov_vel_vert\n"
  "bool pre_flt_fail_mag_field_disturbed\n"
  "\n"
  "uint32 accel_device_id\n"
  "uint32 gyro_device_id\n"
  "uint32 baro_device_id\n"
  "uint32 mag_device_id\n"
  "\n"
  "# legacy local position estimator (LPE) flags\n"
  "uint8 health_flags\\t\\t# Bitmask to indicate sensor health states (vel, pos, hgt)\n"
  "uint8 timeout_flags\\t\\t# Bitmask to indicate timeout flags (vel, pos, hgt)\n"
  "\n"
  "float32 mag_inclination_deg\n"
  "float32 mag_inclination_ref_deg\n"
  "float32 mag_strength_gs\n"
  "float32 mag_strength_ref_gs";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorStatus__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 9225, 9225},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
