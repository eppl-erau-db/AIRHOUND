// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorHygrometer.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_hygrometer__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorHygrometer__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x65, 0x69, 0x88, 0xb0, 0x78, 0xd2, 0x13, 0x85,
      0xf9, 0x7f, 0xd5, 0x79, 0xda, 0xe5, 0x19, 0x67,
      0xf1, 0x88, 0x6f, 0x56, 0x7f, 0x0c, 0x27, 0x71,
      0x7b, 0x59, 0x07, 0xb9, 0x43, 0xd8, 0x37, 0x06,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorHygrometer__TYPE_NAME[] = "px4_msgs/msg/SensorHygrometer";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorHygrometer__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorHygrometer__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__SensorHygrometer__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorHygrometer__FIELD_NAME__temperature[] = "temperature";
static char px4_msgs__msg__SensorHygrometer__FIELD_NAME__humidity[] = "humidity";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorHygrometer__FIELDS[] = {
  {
    {px4_msgs__msg__SensorHygrometer__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorHygrometer__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorHygrometer__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorHygrometer__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorHygrometer__FIELD_NAME__humidity, 8, 8},
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
px4_msgs__msg__SensorHygrometer__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorHygrometer__TYPE_NAME, 29, 29},
      {px4_msgs__msg__SensorHygrometer__FIELDS, 5, 5},
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
  "float32  temperature      # Temperature provided by sensor (Celsius)\n"
  "\n"
  "float32 humidity          # Humidity provided by sensor";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorHygrometer__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorHygrometer__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 321, 321},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorHygrometer__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorHygrometer__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
