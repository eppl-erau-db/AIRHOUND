// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorAccelFifo.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_accel_fifo__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorAccelFifo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0x57, 0x9a, 0x92, 0x87, 0x68, 0xc5, 0x63,
      0x95, 0xe4, 0x0f, 0xe7, 0xdc, 0x79, 0xb1, 0xf6,
      0x4a, 0xb4, 0x91, 0xf7, 0x15, 0xbd, 0x5d, 0x57,
      0x4b, 0xd7, 0x95, 0xbc, 0x5e, 0xca, 0xc3, 0x2d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorAccelFifo__TYPE_NAME[] = "px4_msgs/msg/SensorAccelFifo";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__dt[] = "dt";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__scale[] = "scale";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__samples[] = "samples";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__x[] = "x";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__y[] = "y";
static char px4_msgs__msg__SensorAccelFifo__FIELD_NAME__z[] = "z";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorAccelFifo__FIELDS[] = {
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__dt, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__scale, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__samples, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorAccelFifo__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SensorAccelFifo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorAccelFifo__TYPE_NAME, 28, 28},
      {px4_msgs__msg__SensorAccelFifo__FIELDS, 9, 9},
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
  "float32 dt                # delta time between samples (microseconds)\n"
  "float32 scale\n"
  "\n"
  "uint8 samples             # number of valid samples\n"
  "\n"
  "int16[32] x               # acceleration in the FRD board frame X-axis in m/s^2\n"
  "int16[32] y               # acceleration in the FRD board frame Y-axis in m/s^2\n"
  "int16[32] z               # acceleration in the FRD board frame Z-axis in m/s^2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorAccelFifo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorAccelFifo__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 573, 573},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorAccelFifo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorAccelFifo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
