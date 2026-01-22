// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorSelection.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_selection__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorSelection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd7, 0x98, 0x8f, 0x3d, 0xc4, 0xc4, 0x2a, 0x9d,
      0xb6, 0xab, 0x43, 0xc6, 0xf4, 0x13, 0xc4, 0xe2,
      0x87, 0x98, 0x02, 0x2c, 0x46, 0xe0, 0x9d, 0x1a,
      0xa9, 0x4a, 0x3b, 0x09, 0x32, 0x59, 0x6c, 0x5e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorSelection__TYPE_NAME[] = "px4_msgs/msg/SensorSelection";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorSelection__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorSelection__FIELD_NAME__accel_device_id[] = "accel_device_id";
static char px4_msgs__msg__SensorSelection__FIELD_NAME__gyro_device_id[] = "gyro_device_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorSelection__FIELDS[] = {
  {
    {px4_msgs__msg__SensorSelection__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorSelection__FIELD_NAME__accel_device_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorSelection__FIELD_NAME__gyro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorSelection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorSelection__TYPE_NAME, 28, 28},
      {px4_msgs__msg__SensorSelection__FIELDS, 3, 3},
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
  "# Sensor ID's for the voted sensors output on the sensor_combined topic.\n"
  "# Will be updated on startup of the sensor module and when sensor selection changes\n"
  "#\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint32 accel_device_id\\t\\t# unique device ID for the selected accelerometers\n"
  "uint32 gyro_device_id\\t\\t# unique device ID for the selected rate gyros";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorSelection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorSelection__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 365, 365},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorSelection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorSelection__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
