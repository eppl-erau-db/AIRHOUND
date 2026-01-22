// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorAirflow.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_airflow__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorAirflow__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5e, 0xa8, 0xd2, 0x6b, 0x0f, 0xdd, 0x57, 0x57,
      0xde, 0x4e, 0xa2, 0x33, 0x95, 0x89, 0x4e, 0xe8,
      0x4f, 0x12, 0xb3, 0xa7, 0x0c, 0xe3, 0xca, 0xa4,
      0x16, 0x2c, 0xe8, 0xb6, 0x68, 0x36, 0xe5, 0x6c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorAirflow__TYPE_NAME[] = "px4_msgs/msg/SensorAirflow";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorAirflow__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorAirflow__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorAirflow__FIELD_NAME__speed[] = "speed";
static char px4_msgs__msg__SensorAirflow__FIELD_NAME__direction[] = "direction";
static char px4_msgs__msg__SensorAirflow__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorAirflow__FIELDS[] = {
  {
    {px4_msgs__msg__SensorAirflow__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAirflow__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAirflow__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAirflow__FIELD_NAME__direction, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAirflow__FIELD_NAME__status, 6, 6},
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
px4_msgs__msg__SensorAirflow__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorAirflow__TYPE_NAME, 26, 26},
      {px4_msgs__msg__SensorAirflow__FIELDS, 5, 5},
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
  "uint32 device_id                # unique device ID for the sensor that does not change between power cycles\n"
  "float32 speed\\t\\t\\t# the speed being reported by the wind / airflow sensor\n"
  "float32 direction\\t\\t# the direction being reported by the wind / airflow sensor\n"
  "uint8 status\\t\\t\\t# Status code from the sensor";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorAirflow__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorAirflow__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 363, 363},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorAirflow__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorAirflow__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
