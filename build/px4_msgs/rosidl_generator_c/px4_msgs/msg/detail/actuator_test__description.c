// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ActuatorTest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/actuator_test__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ActuatorTest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0c, 0x3a, 0x10, 0x6c, 0xe1, 0xd7, 0x83, 0xdc,
      0xe2, 0x7d, 0x11, 0xf0, 0xa1, 0x13, 0xff, 0x86,
      0xb5, 0x2e, 0xc1, 0x45, 0x1d, 0xaf, 0x3b, 0xbd,
      0x01, 0xbd, 0xc6, 0xea, 0xa3, 0x75, 0x47, 0x85,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ActuatorTest__TYPE_NAME[] = "px4_msgs/msg/ActuatorTest";

// Define type names, field names, and default values
static char px4_msgs__msg__ActuatorTest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ActuatorTest__FIELD_NAME__action[] = "action";
static char px4_msgs__msg__ActuatorTest__FIELD_NAME__function[] = "function";
static char px4_msgs__msg__ActuatorTest__FIELD_NAME__value[] = "value";
static char px4_msgs__msg__ActuatorTest__FIELD_NAME__timeout_ms[] = "timeout_ms";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ActuatorTest__FIELDS[] = {
  {
    {px4_msgs__msg__ActuatorTest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorTest__FIELD_NAME__action, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorTest__FIELD_NAME__function, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorTest__FIELD_NAME__value, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorTest__FIELD_NAME__timeout_ms, 10, 10},
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
px4_msgs__msg__ActuatorTest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ActuatorTest__TYPE_NAME, 25, 25},
      {px4_msgs__msg__ActuatorTest__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "# Topic to test individual actuator output functions\n"
  "\n"
  "uint8 ACTION_RELEASE_CONTROL = 0\\t# exit test mode for the given function\n"
  "uint8 ACTION_DO_CONTROL = 1\\t\\t\\t# enable actuator test mode\n"
  "\n"
  "uint8 FUNCTION_MOTOR1 = 101\n"
  "uint8 MAX_NUM_MOTORS  = 12\n"
  "uint8 FUNCTION_SERVO1 = 201\n"
  "uint8 MAX_NUM_SERVOS  = 8\n"
  "\n"
  "uint8 action\\t\\t\\t\\t\\t# one of ACTION_*\n"
  "uint16 function\\t\\t\\t\\t\\t# actuator output function\n"
  "float32 value\\t\\t\\t\\t\\t# range: [-1, 1], where 1 means maximum positive output,\n"
  "\\t\\t\\t\\t\\t\\t\\t\\t# 0 to center servos or minimum motor thrust,\n"
  "                   \\t\\t\\t\\t# -1 maximum negative (if not supported by the motors, <0 maps to NaN),\n"
  "                   \\t\\t\\t\\t# and NaN maps to disarmed (stop the motors)\n"
  "uint32 timeout_ms\\t\\t\\t\\t# timeout in ms after which to exit test mode (if 0, do not time out)\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 16                     # >= MAX_NUM_MOTORS to support code in esc_calibration";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ActuatorTest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ActuatorTest__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 927, 927},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ActuatorTest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ActuatorTest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
