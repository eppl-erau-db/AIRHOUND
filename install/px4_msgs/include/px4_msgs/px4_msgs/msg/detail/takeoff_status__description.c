// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TakeoffStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/takeoff_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TakeoffStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x03, 0x99, 0xff, 0x0c, 0x33, 0x0a, 0x42, 0x76,
      0x7f, 0xfd, 0xa3, 0x0f, 0x35, 0x38, 0x00, 0x7c,
      0x40, 0x87, 0x23, 0xc3, 0xd8, 0x82, 0xfb, 0x4d,
      0x8c, 0xeb, 0x17, 0xc8, 0xed, 0x42, 0x93, 0x42,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TakeoffStatus__TYPE_NAME[] = "px4_msgs/msg/TakeoffStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__TakeoffStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TakeoffStatus__FIELD_NAME__takeoff_state[] = "takeoff_state";
static char px4_msgs__msg__TakeoffStatus__FIELD_NAME__tilt_limit[] = "tilt_limit";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TakeoffStatus__FIELDS[] = {
  {
    {px4_msgs__msg__TakeoffStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TakeoffStatus__FIELD_NAME__takeoff_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TakeoffStatus__FIELD_NAME__tilt_limit, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TakeoffStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TakeoffStatus__TYPE_NAME, 26, 26},
      {px4_msgs__msg__TakeoffStatus__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Status of the takeoff state machine currently just available for multicopters\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "uint8 TAKEOFF_STATE_UNINITIALIZED     = 0\n"
  "uint8 TAKEOFF_STATE_DISARMED          = 1\n"
  "uint8 TAKEOFF_STATE_SPOOLUP           = 2\n"
  "uint8 TAKEOFF_STATE_READY_FOR_TAKEOFF = 3\n"
  "uint8 TAKEOFF_STATE_RAMPUP            = 4\n"
  "uint8 TAKEOFF_STATE_FLIGHT            = 5\n"
  "\n"
  "uint8 takeoff_state\n"
  "\n"
  "float32 tilt_limit # limited tilt feasibility during takeoff, contains maximum tilt otherwise";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TakeoffStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TakeoffStatus__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 508, 508},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TakeoffStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TakeoffStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
