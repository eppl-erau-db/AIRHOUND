// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorSensorBias.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_sensor_bias__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorSensorBias__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xab, 0x34, 0x0f, 0x6c, 0xb4, 0x87, 0x05, 0xff,
      0xf8, 0xe9, 0x8a, 0x6a, 0xa5, 0x00, 0xd6, 0x07,
      0x54, 0x67, 0xef, 0x16, 0x58, 0xff, 0xde, 0x6f,
      0x19, 0xec, 0x1a, 0x25, 0x1e, 0xf3, 0xbc, 0x65,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorSensorBias__TYPE_NAME[] = "px4_msgs/msg/EstimatorSensorBias";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_device_id[] = "gyro_device_id";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias[] = "gyro_bias";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_limit[] = "gyro_bias_limit";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_variance[] = "gyro_bias_variance";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_valid[] = "gyro_bias_valid";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_stable[] = "gyro_bias_stable";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_device_id[] = "accel_device_id";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias[] = "accel_bias";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_limit[] = "accel_bias_limit";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_variance[] = "accel_bias_variance";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_valid[] = "accel_bias_valid";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_stable[] = "accel_bias_stable";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_device_id[] = "mag_device_id";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias[] = "mag_bias";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_limit[] = "mag_bias_limit";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_variance[] = "mag_bias_variance";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_valid[] = "mag_bias_valid";
static char px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_stable[] = "mag_bias_stable";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorSensorBias__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_limit, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_variance, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_valid, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__gyro_bias_stable, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_device_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_limit, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_variance, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_valid, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__accel_bias_stable, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_device_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_limit, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_variance, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_valid, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSensorBias__FIELD_NAME__mag_bias_stable, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__EstimatorSensorBias__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorSensorBias__TYPE_NAME, 32, 32},
      {px4_msgs__msg__EstimatorSensorBias__FIELDS, 20, 20},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\n"
  "# Sensor readings and in-run biases in SI-unit form. Sensor readings are compensated for static offsets,\n"
  "# scale errors, in-run bias and thermal drift (if thermal compensation is enabled and available).\n"
  "#\n"
  "\n"
  "uint64 timestamp                # time since system start (microseconds)\n"
  "uint64 timestamp_sample         # the timestamp of the raw data (microseconds)\n"
  "\n"
  "# In-run bias estimates (subtract from uncorrected data)\n"
  "\n"
  "uint32 gyro_device_id           # unique device ID for the sensor that does not change between power cycles\n"
  "float32[3] gyro_bias            # gyroscope in-run bias in body frame (rad/s)\n"
  "float32 gyro_bias_limit         # magnitude of maximum gyroscope in-run bias in body frame (rad/s)\n"
  "float32[3] gyro_bias_variance\n"
  "bool gyro_bias_valid\n"
  "bool gyro_bias_stable\\t\\t# true when the gyro bias estimate is stable enough to use for calibration\n"
  "\n"
  "uint32 accel_device_id          # unique device ID for the sensor that does not change between power cycles\n"
  "float32[3] accel_bias           # accelerometer in-run bias in body frame (m/s^2)\n"
  "float32 accel_bias_limit        # magnitude of maximum accelerometer in-run bias in body frame (m/s^2)\n"
  "float32[3] accel_bias_variance\n"
  "bool accel_bias_valid\n"
  "bool accel_bias_stable\\t\\t# true when the accel bias estimate is stable enough to use for calibration\n"
  "\n"
  "uint32 mag_device_id            # unique device ID for the sensor that does not change between power cycles\n"
  "float32[3] mag_bias             # magnetometer in-run bias in body frame (Gauss)\n"
  "float32 mag_bias_limit          # magnitude of maximum magnetometer in-run bias in body frame (Gauss)\n"
  "float32[3] mag_bias_variance\n"
  "bool mag_bias_valid\n"
  "bool mag_bias_stable\\t\\t# true when the mag bias estimate is stable enough to use for calibration";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorSensorBias__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorSensorBias__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1737, 1737},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorSensorBias__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorSensorBias__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
