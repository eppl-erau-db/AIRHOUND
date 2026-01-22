// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorCombined.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_combined__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorCombined__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe8, 0xbb, 0x56, 0x1e, 0xdb, 0x31, 0x06, 0x31,
      0x50, 0x85, 0x51, 0x92, 0x55, 0xa8, 0x44, 0xe4,
      0x42, 0x06, 0x77, 0xc5, 0x8b, 0xb5, 0xde, 0xce,
      0x8f, 0x7e, 0x30, 0x05, 0xf9, 0xe9, 0x1d, 0x4c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorCombined__TYPE_NAME[] = "px4_msgs/msg/SensorCombined";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorCombined__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_rad[] = "gyro_rad";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_integral_dt[] = "gyro_integral_dt";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_timestamp_relative[] = "accelerometer_timestamp_relative";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_m_s2[] = "accelerometer_m_s2";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_integral_dt[] = "accelerometer_integral_dt";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_clipping[] = "accelerometer_clipping";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_clipping[] = "gyro_clipping";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__accel_calibration_count[] = "accel_calibration_count";
static char px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_calibration_count[] = "gyro_calibration_count";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorCombined__FIELDS[] = {
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_rad, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_integral_dt, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_timestamp_relative, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_m_s2, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_integral_dt, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__accelerometer_clipping, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_clipping, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__accel_calibration_count, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCombined__FIELD_NAME__gyro_calibration_count, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorCombined__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorCombined__TYPE_NAME, 27, 27},
      {px4_msgs__msg__SensorCombined__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Sensor readings in SI-unit form.\n"
  "# These fields are scaled and offset-compensated where possible and do not\n"
  "# change with board revisions and sensor updates.\n"
  "\n"
  "uint64 timestamp                                # time since system start (microseconds)\n"
  "\n"
  "int32 RELATIVE_TIMESTAMP_INVALID = 2147483647   # (0x7fffffff) If one of the relative timestamps is set to this value, it means the associated sensor values are invalid\n"
  "\n"
  "# gyro timstamp is equal to the timestamp of the message\n"
  "float32[3] gyro_rad                     # average angular rate measured in the FRD body frame XYZ-axis in rad/s over the last gyro sampling period\n"
  "uint32 gyro_integral_dt                 # gyro measurement sampling period in microseconds\n"
  "\n"
  "int32 accelerometer_timestamp_relative  # timestamp + accelerometer_timestamp_relative = Accelerometer timestamp\n"
  "float32[3] accelerometer_m_s2           # average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period\n"
  "uint32 accelerometer_integral_dt        # accelerometer measurement sampling period in microseconds\n"
  "\n"
  "uint8 CLIPPING_X = 1\n"
  "uint8 CLIPPING_Y = 2\n"
  "uint8 CLIPPING_Z = 4\n"
  "\n"
  "uint8 accelerometer_clipping    # bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame\n"
  "uint8 gyro_clipping             # bitfield indicating if there was any gyro clipping (per axis) during the integration time frame\n"
  "\n"
  "uint8 accel_calibration_count   # Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.\n"
  "uint8 gyro_calibration_count    # Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorCombined__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorCombined__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1684, 1684},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorCombined__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorCombined__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
