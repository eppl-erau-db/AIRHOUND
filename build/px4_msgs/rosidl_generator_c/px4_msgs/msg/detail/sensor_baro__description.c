// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorBaro.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_baro__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorBaro__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x12, 0x9a, 0xd6, 0x43, 0xf5, 0x83, 0x52, 0x90,
      0xe3, 0x30, 0x8b, 0x53, 0x21, 0xb2, 0x29, 0xf9,
      0x87, 0x1d, 0x4b, 0x08, 0x55, 0xf7, 0x0a, 0xa3,
      0xfe, 0xa5, 0x26, 0x27, 0xba, 0x4a, 0x51, 0x16,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorBaro__TYPE_NAME[] = "px4_msgs/msg/SensorBaro";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorBaro__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorBaro__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__SensorBaro__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorBaro__FIELD_NAME__pressure[] = "pressure";
static char px4_msgs__msg__SensorBaro__FIELD_NAME__temperature[] = "temperature";
static char px4_msgs__msg__SensorBaro__FIELD_NAME__error_count[] = "error_count";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorBaro__FIELDS[] = {
  {
    {px4_msgs__msg__SensorBaro__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorBaro__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorBaro__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorBaro__FIELD_NAME__pressure, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorBaro__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorBaro__FIELD_NAME__error_count, 11, 11},
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
px4_msgs__msg__SensorBaro__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorBaro__TYPE_NAME, 23, 23},
      {px4_msgs__msg__SensorBaro__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp          # time since system start (microseconds)\n"
  "uint64 timestamp_sample\n"
  "\n"
  "uint32 device_id          # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "float32 pressure          # static pressure measurement in Pascals\n"
  "\n"
  "float32 temperature       # temperature in degrees Celsius\n"
  "\n"
  "uint32 error_count\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 4";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorBaro__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorBaro__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 370, 370},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorBaro__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorBaro__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
