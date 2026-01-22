// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GpioIn.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gpio_in__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GpioIn__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0e, 0xd8, 0xa6, 0x8c, 0xd6, 0xc8, 0xf8, 0x92,
      0xf1, 0x16, 0x41, 0xa8, 0xe2, 0x02, 0x17, 0x80,
      0x23, 0xa6, 0x60, 0x86, 0xdb, 0xce, 0xdf, 0x13,
      0x4d, 0x19, 0xd8, 0xb2, 0x95, 0x31, 0xc1, 0x90,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GpioIn__TYPE_NAME[] = "px4_msgs/msg/GpioIn";

// Define type names, field names, and default values
static char px4_msgs__msg__GpioIn__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GpioIn__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__GpioIn__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GpioIn__FIELDS[] = {
  {
    {px4_msgs__msg__GpioIn__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpioIn__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpioIn__FIELD_NAME__state, 5, 5},
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
px4_msgs__msg__GpioIn__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GpioIn__TYPE_NAME, 19, 19},
      {px4_msgs__msg__GpioIn__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GPIO mask and state\n"
  "\n"
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "uint32 device_id\\t\\t\\t# Device id\n"
  "\n"
  "uint32 state\\t\\t\\t\\t# pin state mask";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GpioIn__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GpioIn__TYPE_NAME, 19, 19},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 148, 148},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GpioIn__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GpioIn__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
