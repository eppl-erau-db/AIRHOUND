// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorsStatusImu.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensors_status_imu__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorsStatusImu__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0xd1, 0x0a, 0xd4, 0x78, 0xf7, 0x0c, 0x61,
      0x33, 0x9f, 0xab, 0xf4, 0xd1, 0xc2, 0x09, 0x09,
      0xe0, 0x17, 0x1f, 0x11, 0xb3, 0xc9, 0x1e, 0xc1,
      0x79, 0x5c, 0x27, 0x5f, 0x5a, 0x1a, 0x11, 0x32,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorsStatusImu__TYPE_NAME[] = "px4_msgs/msg/SensorsStatusImu";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_device_id_primary[] = "accel_device_id_primary";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_device_ids[] = "accel_device_ids";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_inconsistency_m_s_s[] = "accel_inconsistency_m_s_s";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_healthy[] = "accel_healthy";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_priority[] = "accel_priority";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_device_id_primary[] = "gyro_device_id_primary";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_device_ids[] = "gyro_device_ids";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_inconsistency_rad_s[] = "gyro_inconsistency_rad_s";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_healthy[] = "gyro_healthy";
static char px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_priority[] = "gyro_priority";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorsStatusImu__FIELDS[] = {
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_device_id_primary, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_device_ids, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_inconsistency_m_s_s, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_healthy, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__accel_priority, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_device_id_primary, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_device_ids, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_inconsistency_rad_s, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_healthy, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorsStatusImu__FIELD_NAME__gyro_priority, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorsStatusImu__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorsStatusImu__TYPE_NAME, 29, 29},
      {px4_msgs__msg__SensorsStatusImu__FIELDS, 11, 11},
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
  "# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.\n"
  "#\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "uint32 accel_device_id_primary       # current primary accel device id for reference\n"
  "\n"
  "uint32[4] accel_device_ids\n"
  "float32[4] accel_inconsistency_m_s_s # magnitude of acceleration difference between IMU instance and mean in m/s^2.\n"
  "bool[4] accel_healthy\n"
  "uint8[4] accel_priority\n"
  "\n"
  "uint32 gyro_device_id_primary       # current primary gyro device id for reference\n"
  "\n"
  "uint32[4] gyro_device_ids\n"
  "float32[4] gyro_inconsistency_rad_s # magnitude of angular rate difference between IMU instance and mean in (rad/s).\n"
  "bool[4] gyro_healthy\n"
  "uint8[4] gyro_priority";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorsStatusImu__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorsStatusImu__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 696, 696},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorsStatusImu__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorsStatusImu__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
