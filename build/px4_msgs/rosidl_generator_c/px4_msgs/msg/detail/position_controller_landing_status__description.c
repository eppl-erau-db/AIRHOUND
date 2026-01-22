// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/PositionControllerLandingStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/position_controller_landing_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__PositionControllerLandingStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfa, 0x51, 0xcb, 0xd6, 0x53, 0x9d, 0x21, 0x14,
      0x9e, 0x8f, 0xf8, 0xd2, 0x46, 0xec, 0x83, 0xab,
      0x38, 0x69, 0xb4, 0x1a, 0xe7, 0x49, 0xe9, 0x68,
      0x72, 0x3a, 0x47, 0x80, 0xf5, 0x85, 0x06, 0xfb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__PositionControllerLandingStatus__TYPE_NAME[] = "px4_msgs/msg/PositionControllerLandingStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__lateral_touchdown_offset[] = "lateral_touchdown_offset";
static char px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__flaring[] = "flaring";
static char px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__abort_status[] = "abort_status";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__PositionControllerLandingStatus__FIELDS[] = {
  {
    {px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__lateral_touchdown_offset, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__flaring, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PositionControllerLandingStatus__FIELD_NAME__abort_status, 12, 12},
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
px4_msgs__msg__PositionControllerLandingStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__PositionControllerLandingStatus__TYPE_NAME, 44, 44},
      {px4_msgs__msg__PositionControllerLandingStatus__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # [us] time since system start\n"
  "float32 lateral_touchdown_offset # [m] lateral touchdown position offset manually commanded during landing\n"
  "bool flaring # true if the aircraft is flaring\n"
  "\n"
  "# abort status is:\n"
  "# 0 if not aborted\n"
  "# >0 if aborted, with the singular abort criterion which triggered the landing abort enumerated by the following abort reasons\n"
  "uint8 abort_status\n"
  "\n"
  "# abort reasons\n"
  "# after the manual operator abort, corresponds to individual bits of param FW_LND_ABORT\n"
  "uint8 NOT_ABORTED = 0\n"
  "uint8 ABORTED_BY_OPERATOR = 1\n"
  "uint8 TERRAIN_NOT_FOUND = 2 # FW_LND_ABORT (1 << 0)\n"
  "uint8 TERRAIN_TIMEOUT = 3 # FW_LND_ABORT (1 << 1)\n"
  "uint8 UNKNOWN_ABORT_CRITERION = 4";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__PositionControllerLandingStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__PositionControllerLandingStatus__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 680, 680},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__PositionControllerLandingStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__PositionControllerLandingStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
