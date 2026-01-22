// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ActuatorMotors.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/actuator_motors__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ActuatorMotors__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x78, 0x57, 0x3a, 0xb1, 0xd2, 0x6b, 0x08, 0xf3,
      0xf4, 0x94, 0xa1, 0x64, 0x76, 0xcc, 0x89, 0x6a,
      0xf5, 0xf8, 0xdc, 0xeb, 0x64, 0xf6, 0xf7, 0x23,
      0xb4, 0xa2, 0x53, 0x3f, 0xf2, 0xfb, 0xd3, 0x21,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ActuatorMotors__TYPE_NAME[] = "px4_msgs/msg/ActuatorMotors";

// Define type names, field names, and default values
static char px4_msgs__msg__ActuatorMotors__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ActuatorMotors__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__ActuatorMotors__FIELD_NAME__reversible_flags[] = "reversible_flags";
static char px4_msgs__msg__ActuatorMotors__FIELD_NAME__control[] = "control";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ActuatorMotors__FIELDS[] = {
  {
    {px4_msgs__msg__ActuatorMotors__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorMotors__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorMotors__FIELD_NAME__reversible_flags, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorMotors__FIELD_NAME__control, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      12,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ActuatorMotors__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ActuatorMotors__TYPE_NAME, 27, 27},
      {px4_msgs__msg__ActuatorMotors__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Motor control message\n"
  "#\n"
  "# Normalised thrust setpoint for up to 12 motors.\n"
  "# Published by the vehicle's allocation and consumed by the ESC protocol drivers e.g. PWM, DSHOT, UAVCAN.\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp         # [us] Time since system start\n"
  "uint64 timestamp_sample  # [us] Sampling timestamp of the data this control response is based on\n"
  "\n"
  "uint16 reversible_flags  # [-] Bitset indicating which motors are configured to be reversible\n"
  "\n"
  "uint8 ACTUATOR_FUNCTION_MOTOR1 = 101 #\n"
  "\n"
  "uint8 NUM_CONTROLS = 12  #\n"
  "float32[12] control      # [@range -1, 1] Normalized thrust. where 1 means maximum positive thrust, -1 maximum negative (if not supported by the output, <0 maps to NaN). NaN maps to disarmed (stop the motors)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ActuatorMotors__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ActuatorMotors__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 736, 736},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ActuatorMotors__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ActuatorMotors__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
