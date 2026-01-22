// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleImuStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_imu_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleImuStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfc, 0x22, 0x18, 0x50, 0x2e, 0xfa, 0x24, 0x0e,
      0x1d, 0x43, 0xe9, 0x7a, 0x4d, 0x8e, 0x1c, 0x08,
      0x7c, 0xbb, 0x76, 0xfc, 0x8c, 0xec, 0xd1, 0x5b,
      0xce, 0xc3, 0x06, 0x41, 0x59, 0x3b, 0x0d, 0xa0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleImuStatus__TYPE_NAME[] = "px4_msgs/msg/VehicleImuStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_device_id[] = "accel_device_id";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_device_id[] = "gyro_device_id";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_clipping[] = "accel_clipping";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_clipping[] = "gyro_clipping";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_error_count[] = "accel_error_count";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_error_count[] = "gyro_error_count";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_rate_hz[] = "accel_rate_hz";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_rate_hz[] = "gyro_rate_hz";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_raw_rate_hz[] = "accel_raw_rate_hz";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_raw_rate_hz[] = "gyro_raw_rate_hz";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_vibration_metric[] = "accel_vibration_metric";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_vibration_metric[] = "gyro_vibration_metric";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__delta_angle_coning_metric[] = "delta_angle_coning_metric";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__mean_accel[] = "mean_accel";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__mean_gyro[] = "mean_gyro";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__var_accel[] = "var_accel";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__var_gyro[] = "var_gyro";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__temperature_accel[] = "temperature_accel";
static char px4_msgs__msg__VehicleImuStatus__FIELD_NAME__temperature_gyro[] = "temperature_gyro";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleImuStatus__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_device_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_clipping, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_clipping, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_error_count, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_error_count, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_rate_hz, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_rate_hz, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_raw_rate_hz, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_raw_rate_hz, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__accel_vibration_metric, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__gyro_vibration_metric, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__delta_angle_coning_metric, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__mean_accel, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__mean_gyro, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__var_accel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__var_gyro, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__temperature_accel, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleImuStatus__FIELD_NAME__temperature_gyro, 16, 16},
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
px4_msgs__msg__VehicleImuStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleImuStatus__TYPE_NAME, 29, 29},
      {px4_msgs__msg__VehicleImuStatus__FIELDS, 20, 20},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                # time since system start (microseconds)\n"
  "\n"
  "uint32 accel_device_id          # unique device ID for the sensor that does not change between power cycles\n"
  "uint32 gyro_device_id           # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "uint32[3] accel_clipping        # total clipping per axis\n"
  "uint32[3] gyro_clipping         # total clipping per axis\n"
  "\n"
  "uint32 accel_error_count\n"
  "uint32 gyro_error_count\n"
  "\n"
  "float32 accel_rate_hz\n"
  "float32 gyro_rate_hz\n"
  "\n"
  "float32 accel_raw_rate_hz       # full raw sensor sample rate (Hz)\n"
  "float32 gyro_raw_rate_hz        # full raw sensor sample rate (Hz)\n"
  "\n"
  "float32 accel_vibration_metric  # high frequency vibration level in the accelerometer data (m/s/s)\n"
  "float32 gyro_vibration_metric   # high frequency vibration level in the gyro data (rad/s)\n"
  "float32 delta_angle_coning_metric # average IMU delta angle coning correction (rad^2)\n"
  "\n"
  "float32[3] mean_accel           # average accelerometer readings since last publication\n"
  "float32[3] mean_gyro            # average gyroscope readings since last publication\n"
  "float32[3] var_accel            # accelerometer variance since last publication\n"
  "float32[3] var_gyro             # gyroscope variance since last publication\n"
  "\n"
  "float32 temperature_accel\n"
  "float32 temperature_gyro";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleImuStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleImuStatus__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1293, 1293},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleImuStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleImuStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
