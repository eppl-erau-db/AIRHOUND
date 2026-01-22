// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/CanInterfaceStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/can_interface_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__CanInterfaceStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdd, 0x3f, 0xfb, 0x8c, 0x95, 0x45, 0x90, 0xa7,
      0x32, 0x78, 0x48, 0x67, 0xf2, 0xa0, 0xe8, 0xcd,
      0xe0, 0x36, 0xc0, 0xdc, 0xbd, 0x21, 0xe3, 0x85,
      0xfa, 0xf6, 0x55, 0x76, 0x02, 0xa5, 0xba, 0xe4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__CanInterfaceStatus__TYPE_NAME[] = "px4_msgs/msg/CanInterfaceStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__interface[] = "interface";
static char px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__io_errors[] = "io_errors";
static char px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__frames_tx[] = "frames_tx";
static char px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__frames_rx[] = "frames_rx";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__CanInterfaceStatus__FIELDS[] = {
  {
    {px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__interface, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__io_errors, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__frames_tx, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CanInterfaceStatus__FIELD_NAME__frames_rx, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__CanInterfaceStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__CanInterfaceStatus__TYPE_NAME, 31, 31},
      {px4_msgs__msg__CanInterfaceStatus__FIELDS, 5, 5},
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
  "uint8 interface\n"
  "\n"
  "uint64 io_errors\n"
  "uint64 frames_tx\n"
  "uint64 frames_rx";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__CanInterfaceStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__CanInterfaceStatus__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 127, 127},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__CanInterfaceStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__CanInterfaceStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
