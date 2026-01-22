// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GpioConfig.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gpio_config__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GpioConfig__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf2, 0x47, 0x4c, 0xb3, 0x1f, 0x3e, 0x6c, 0xa0,
      0xc3, 0x48, 0xca, 0x62, 0x25, 0x6a, 0xea, 0x9f,
      0xda, 0x70, 0x12, 0x99, 0x34, 0x86, 0x24, 0x51,
      0xf3, 0xe9, 0xc1, 0x36, 0x32, 0x73, 0xdf, 0x40,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GpioConfig__TYPE_NAME[] = "px4_msgs/msg/GpioConfig";

// Define type names, field names, and default values
static char px4_msgs__msg__GpioConfig__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GpioConfig__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__GpioConfig__FIELD_NAME__mask[] = "mask";
static char px4_msgs__msg__GpioConfig__FIELD_NAME__state[] = "state";
static char px4_msgs__msg__GpioConfig__FIELD_NAME__config[] = "config";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GpioConfig__FIELDS[] = {
  {
    {px4_msgs__msg__GpioConfig__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpioConfig__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpioConfig__FIELD_NAME__mask, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpioConfig__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpioConfig__FIELD_NAME__config, 6, 6},
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
px4_msgs__msg__GpioConfig__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GpioConfig__TYPE_NAME, 23, 23},
      {px4_msgs__msg__GpioConfig__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GPIO configuration\n"
  "\n"
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "uint32 device_id\\t\\t\\t# Device id\n"
  "\n"
  "uint32 mask\\t\\t\\t\\t# Pin mask\n"
  "uint32 state\\t\\t\\t\\t# Initial pin output state\n"
  "\n"
  "# Configuration Mask\n"
  "# Bit 0-3: Direction: 0=Input, 1=Output\n"
  "# Bit 4-7: Input Config: 0=Floating, 1=PullUp, 2=PullDown\n"
  "# Bit 8-12: Output Config: 0=PushPull, 1=OpenDrain\n"
  "# Bit 13-31: Reserved\n"
  "uint32 INPUT\\t\\t\\t= 0\\t# 0x0000\n"
  "uint32 OUTPUT\\t\\t\\t= 1\\t# 0x0001\n"
  "uint32 PULLUP\\t\\t\\t= 16\\t# 0x0010\n"
  "uint32 PULLDOWN\\t\\t\\t= 32\\t# 0x0020\n"
  "uint32 OPENDRAIN\\t\\t= 256\\t# 0x0100\n"
  "\n"
  "uint32 INPUT_FLOATING\\t\\t= 0\\t# 0x0000\n"
  "uint32 INPUT_PULLUP\\t\\t= 16\\t# 0x0010\n"
  "uint32 INPUT_PULLDOWN\\t\\t= 32\\t# 0x0020\n"
  "\n"
  "uint32 OUTPUT_PUSHPULL\\t\\t= 0\\t# 0x0000\n"
  "uint32 OUTPUT_OPENDRAIN\\t\\t= 256\\t# 0x0100\n"
  "uint32 OUTPUT_OPENDRAIN_PULLUP\\t= 272\\t# 0x0110\n"
  "\n"
  "uint32 config";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GpioConfig__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GpioConfig__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 776, 776},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GpioConfig__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GpioConfig__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
