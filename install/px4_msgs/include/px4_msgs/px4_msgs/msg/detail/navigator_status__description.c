// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/NavigatorStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/navigator_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__NavigatorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0x2e, 0x07, 0x63, 0x7b, 0x9e, 0x15, 0xb3,
      0x2c, 0x7e, 0x4e, 0x37, 0xae, 0x48, 0xc9, 0xcd,
      0xc8, 0x16, 0x83, 0xf5, 0x0e, 0x79, 0xf7, 0x71,
      0x17, 0xb2, 0x76, 0x2a, 0x88, 0x66, 0x3a, 0x88,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__NavigatorStatus__TYPE_NAME[] = "px4_msgs/msg/NavigatorStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__NavigatorStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__NavigatorStatus__FIELD_NAME__nav_state[] = "nav_state";
static char px4_msgs__msg__NavigatorStatus__FIELD_NAME__failure[] = "failure";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__NavigatorStatus__FIELDS[] = {
  {
    {px4_msgs__msg__NavigatorStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorStatus__FIELD_NAME__nav_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorStatus__FIELD_NAME__failure, 7, 7},
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
px4_msgs__msg__NavigatorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__NavigatorStatus__TYPE_NAME, 28, 28},
      {px4_msgs__msg__NavigatorStatus__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Current status of a Navigator mode\n"
  "# The possible values of nav_state are defined in the VehicleStatus msg.\n"
  "uint64 timestamp  # time since system start (microseconds)\n"
  "\n"
  "uint8 nav_state   # Source mode (values in VehicleStatus)\n"
  "uint8 failure     # Navigator failure enum\n"
  "\n"
  "uint8 FAILURE_NONE = 0\n"
  "uint8 FAILURE_HAGL = 1 # Target altitude exceeds maximum height above ground";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__NavigatorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__NavigatorStatus__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 372, 372},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__NavigatorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__NavigatorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
