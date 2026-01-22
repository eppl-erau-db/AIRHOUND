// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LaunchDetectionStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/launch_detection_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LaunchDetectionStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xeb, 0x00, 0xb4, 0x4a, 0x7d, 0xde, 0x38, 0x12,
      0xfc, 0xf2, 0x96, 0xba, 0xbf, 0xa1, 0xf2, 0x62,
      0x6c, 0x55, 0x66, 0x0d, 0x43, 0xbd, 0xbe, 0x49,
      0x24, 0xe3, 0xb6, 0x1c, 0x17, 0x38, 0xf3, 0x02,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LaunchDetectionStatus__TYPE_NAME[] = "px4_msgs/msg/LaunchDetectionStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__LaunchDetectionStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LaunchDetectionStatus__FIELD_NAME__launch_detection_state[] = "launch_detection_state";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LaunchDetectionStatus__FIELDS[] = {
  {
    {px4_msgs__msg__LaunchDetectionStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LaunchDetectionStatus__FIELD_NAME__launch_detection_state, 22, 22},
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
px4_msgs__msg__LaunchDetectionStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LaunchDetectionStatus__TYPE_NAME, 34, 34},
      {px4_msgs__msg__LaunchDetectionStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Status of the launch detection state machine (fixed-wing only)\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "uint8 STATE_WAITING_FOR_LAUNCH \\t\\t\\t= 0 # waiting for launch\n"
  "uint8 STATE_LAUNCH_DETECTED_DISABLED_MOTOR \\t= 1 # launch detected, but keep motor(s) disabled (e.g. because it can't spin freely while on catapult)\n"
  "uint8 STATE_FLYING \\t\\t\\t\\t= 2 # launch detected, use normal takeoff/flying configuration\n"
  "\n"
  "uint8 launch_detection_state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LaunchDetectionStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LaunchDetectionStatus__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 448, 448},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LaunchDetectionStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LaunchDetectionStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
