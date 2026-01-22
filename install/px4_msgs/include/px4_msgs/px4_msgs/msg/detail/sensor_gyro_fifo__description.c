// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorGyroFifo.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_gyro_fifo__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorGyroFifo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x58, 0x6e, 0x39, 0xf7, 0x76, 0x7b, 0xce, 0xfc,
      0x66, 0x8e, 0xb4, 0xe4, 0x40, 0xaf, 0x79, 0xd6,
      0x03, 0xf4, 0xd5, 0x39, 0x6a, 0x93, 0x89, 0xb3,
      0xfb, 0xfb, 0x3d, 0x35, 0x2e, 0x77, 0x62, 0x01,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorGyroFifo__TYPE_NAME[] = "px4_msgs/msg/SensorGyroFifo";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__dt[] = "dt";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__scale[] = "scale";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__samples[] = "samples";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__x[] = "x";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__y[] = "y";
static char px4_msgs__msg__SensorGyroFifo__FIELD_NAME__z[] = "z";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorGyroFifo__FIELDS[] = {
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__dt, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__scale, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__samples, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGyroFifo__FIELD_NAME__z, 1, 1},
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
px4_msgs__msg__SensorGyroFifo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorGyroFifo__TYPE_NAME, 27, 27},
      {px4_msgs__msg__SensorGyroFifo__FIELDS, 9, 9},
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
  "int16[32] x               # angular velocity in the FRD board frame X-axis in rad/s\n"
  "int16[32] y               # angular velocity in the FRD board frame Y-axis in rad/s\n"
  "int16[32] z               # angular velocity in the FRD board frame Z-axis in rad/s\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 4";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorGyroFifo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorGyroFifo__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 613, 613},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorGyroFifo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorGyroFifo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
