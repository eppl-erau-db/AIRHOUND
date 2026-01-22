// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GpioRequest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gpio_request__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GpioRequest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x42, 0xb5, 0xe4, 0xf6, 0x8b, 0xc8, 0x67, 0x9c,
      0x85, 0x50, 0x7c, 0xa8, 0xfe, 0xef, 0x73, 0x2e,
      0x54, 0x52, 0x08, 0x88, 0x52, 0x6f, 0xdf, 0xe6,
      0x37, 0xbf, 0x31, 0xa7, 0xd9, 0x2d, 0x03, 0x0f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GpioRequest__TYPE_NAME[] = "px4_msgs/msg/GpioRequest";

// Define type names, field names, and default values
static char px4_msgs__msg__GpioRequest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GpioRequest__FIELD_NAME__device_id[] = "device_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GpioRequest__FIELDS[] = {
  {
    {px4_msgs__msg__GpioRequest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpioRequest__FIELD_NAME__device_id, 9, 9},
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
px4_msgs__msg__GpioRequest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GpioRequest__TYPE_NAME, 24, 24},
      {px4_msgs__msg__GpioRequest__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request GPIO mask to be read\n"
  "\n"
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "uint32 device_id\\t\\t\\t# Device id";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GpioRequest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GpioRequest__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 123, 123},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GpioRequest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GpioRequest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
