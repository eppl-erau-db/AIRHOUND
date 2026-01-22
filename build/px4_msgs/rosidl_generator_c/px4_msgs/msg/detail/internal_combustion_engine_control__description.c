// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/InternalCombustionEngineControl.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/internal_combustion_engine_control__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__InternalCombustionEngineControl__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x54, 0xe6, 0xc1, 0x83, 0x74, 0x25, 0x59, 0x85,
      0x59, 0x71, 0xa9, 0x50, 0xc1, 0x69, 0x2a, 0x57,
      0xbf, 0x9d, 0xe0, 0xca, 0x3a, 0xeb, 0xa9, 0x26,
      0x5d, 0x0d, 0xd0, 0x7c, 0x29, 0x32, 0xf2, 0x8a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__InternalCombustionEngineControl__TYPE_NAME[] = "px4_msgs/msg/InternalCombustionEngineControl";

// Define type names, field names, and default values
static char px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__ignition_on[] = "ignition_on";
static char px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__throttle_control[] = "throttle_control";
static char px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__choke_control[] = "choke_control";
static char px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__starter_engine_control[] = "starter_engine_control";
static char px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__user_request[] = "user_request";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__InternalCombustionEngineControl__FIELDS[] = {
  {
    {px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__ignition_on, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__throttle_control, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__choke_control, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__starter_engine_control, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InternalCombustionEngineControl__FIELD_NAME__user_request, 12, 12},
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
px4_msgs__msg__InternalCombustionEngineControl__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__InternalCombustionEngineControl__TYPE_NAME, 44, 44},
      {px4_msgs__msg__InternalCombustionEngineControl__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp        \\t\\t# time since system start (microseconds)\n"
  "\n"
  "bool ignition_on          \\t\\t# activate/deactivate ignition (spark plug)\n"
  "float32 throttle_control\\t\\t# setpoint for throttle actuator, with slew rate if enabled, idles with 0 [norm] [@range 0,1] [@uncontrolled NAN to stop motor]\n"
  "float32 choke_control\\t\\t\\t# setpoint for choke actuator, 1: fully closed [norm] [@range 0,1]\n"
  "float32 starter_engine_control\\t\\t# setpoint for (electric) starter motor [norm] [@range 0,1]\n"
  "\n"
  "uint8 user_request\\t\\t\\t# user intent for the ICE being on/off";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__InternalCombustionEngineControl__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__InternalCombustionEngineControl__TYPE_NAME, 44, 44},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 538, 538},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__InternalCombustionEngineControl__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__InternalCombustionEngineControl__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
