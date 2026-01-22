// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Gripper.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gripper__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Gripper__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1d, 0x71, 0x27, 0xee, 0x07, 0xb7, 0x17, 0x72,
      0xcb, 0x12, 0xc0, 0x10, 0x99, 0x5a, 0x08, 0x77,
      0x81, 0xb2, 0x7b, 0x65, 0x09, 0x38, 0x71, 0x6b,
      0x1c, 0xaf, 0xad, 0xa4, 0x41, 0x06, 0xa3, 0x2b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Gripper__TYPE_NAME[] = "px4_msgs/msg/Gripper";

// Define type names, field names, and default values
static char px4_msgs__msg__Gripper__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Gripper__FIELD_NAME__command[] = "command";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Gripper__FIELDS[] = {
  {
    {px4_msgs__msg__Gripper__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Gripper__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__Gripper__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Gripper__TYPE_NAME, 20, 20},
      {px4_msgs__msg__Gripper__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "## Used to command an actuation in the gripper, which is mapped to a specific output in the control allocation module\n"
  "\n"
  "uint64 timestamp\n"
  "\n"
  "int8 command\\t\\t# Commanded state for the gripper\n"
  "int8 COMMAND_GRAB = 0\n"
  "int8 COMMAND_RELEASE = 1";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Gripper__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Gripper__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 232, 232},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Gripper__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Gripper__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
