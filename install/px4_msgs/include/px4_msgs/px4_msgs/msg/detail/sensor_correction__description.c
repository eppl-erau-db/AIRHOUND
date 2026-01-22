// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorCorrection.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_correction__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorCorrection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x30, 0x81, 0x96, 0xfe, 0x5e, 0xe9, 0xcf, 0xc3,
      0x76, 0x96, 0x20, 0x8d, 0xbb, 0x17, 0x54, 0x64,
      0x76, 0xe8, 0x91, 0x59, 0xb6, 0x83, 0xf8, 0x84,
      0xf9, 0x91, 0x19, 0x0b, 0x78, 0xd3, 0xf0, 0xde,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorCorrection__TYPE_NAME[] = "px4_msgs/msg/SensorCorrection";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_device_ids[] = "accel_device_ids";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_temperature[] = "accel_temperature";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_0[] = "accel_offset_0";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_1[] = "accel_offset_1";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_2[] = "accel_offset_2";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_3[] = "accel_offset_3";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_device_ids[] = "gyro_device_ids";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_temperature[] = "gyro_temperature";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_0[] = "gyro_offset_0";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_1[] = "gyro_offset_1";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_2[] = "gyro_offset_2";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_3[] = "gyro_offset_3";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_device_ids[] = "mag_device_ids";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_temperature[] = "mag_temperature";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_0[] = "mag_offset_0";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_1[] = "mag_offset_1";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_2[] = "mag_offset_2";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_3[] = "mag_offset_3";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_device_ids[] = "baro_device_ids";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_temperature[] = "baro_temperature";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_0[] = "baro_offset_0";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_1[] = "baro_offset_1";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_2[] = "baro_offset_2";
static char px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_3[] = "baro_offset_3";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorCorrection__FIELDS[] = {
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_device_ids, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_temperature, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_0, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_1, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_2, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__accel_offset_3, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_device_ids, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_temperature, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_0, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_1, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_2, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__gyro_offset_3, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_device_ids, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_temperature, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_0, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_1, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_2, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__mag_offset_3, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_device_ids, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_temperature, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_0, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_1, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_2, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorCorrection__FIELD_NAME__baro_offset_3, 13, 13},
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
px4_msgs__msg__SensorCorrection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorCorrection__TYPE_NAME, 29, 29},
      {px4_msgs__msg__SensorCorrection__FIELDS, 25, 25},
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
  "# Sensor corrections in SI-unit form for the voted sensor\n"
  "#\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "# Corrections for acceleromter acceleration outputs where corrected_accel = raw_accel * accel_scale + accel_offset\n"
  "# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame\n"
  "uint32[4] accel_device_ids\n"
  "float32[4] accel_temperature\n"
  "float32[3] accel_offset_0\\t# accelerometer 0 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "float32[3] accel_offset_1\\t# accelerometer 1 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "float32[3] accel_offset_2\\t# accelerometer 2 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "float32[3] accel_offset_3\\t# accelerometer 3 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "\n"
  "# Corrections for gyro angular rate outputs where corrected_rate = raw_rate * gyro_scale + gyro_offset\n"
  "# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame\n"
  "uint32[4] gyro_device_ids\n"
  "float32[4] gyro_temperature\n"
  "float32[3] gyro_offset_0\\t# gyro 0 XYZ offsets in the sensor frame in rad/s\n"
  "float32[3] gyro_offset_1\\t# gyro 1 XYZ offsets in the sensor frame in rad/s\n"
  "float32[3] gyro_offset_2\\t# gyro 2 XYZ offsets in the sensor frame in rad/s\n"
  "float32[3] gyro_offset_3\\t# gyro 3 XYZ offsets in the sensor frame in rad/s\n"
  "\n"
  "# Corrections for magnetometer measurement outputs where corrected_mag = raw_mag * mag_scale + mag_offset\n"
  "# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame\n"
  "uint32[4] mag_device_ids\n"
  "float32[4] mag_temperature\n"
  "float32[3] mag_offset_0\\t# magnetometer 0 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "float32[3] mag_offset_1\\t# magnetometer 1 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "float32[3] mag_offset_2\\t# magnetometer 2 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "float32[3] mag_offset_3\\t# magnetometer 3 offsets in the FRD board frame XYZ-axis in m/s^s\n"
  "\n"
  "# Corrections for barometric pressure outputs where corrected_pressure = raw_pressure * pressure_scale + pressure_offset\n"
  "# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame\n"
  "uint32[4] baro_device_ids\n"
  "float32[4] baro_temperature\n"
  "float32 baro_offset_0\\t\\t# barometric pressure 0 offsets in the sensor frame in Pascals\n"
  "float32 baro_offset_1\\t\\t# barometric pressure 1 offsets in the sensor frame in Pascals\n"
  "float32 baro_offset_2\\t\\t# barometric pressure 2 offsets in the sensor frame in Pascals\n"
  "float32 baro_offset_3\\t\\t# barometric pressure 3 offsets in the sensor frame in Pascals";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorCorrection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorCorrection__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2631, 2631},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorCorrection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorCorrection__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
