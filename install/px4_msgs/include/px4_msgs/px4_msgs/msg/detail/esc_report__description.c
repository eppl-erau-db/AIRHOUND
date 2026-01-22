// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EscReport.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/esc_report__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EscReport__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x03, 0x53, 0xf5, 0x26, 0xdc, 0xa2, 0x76, 0x3e,
      0x2a, 0xe2, 0x62, 0xb6, 0xda, 0xae, 0xda, 0x11,
      0x9b, 0x3d, 0x1e, 0x9d, 0xa1, 0x2c, 0x0f, 0x49,
      0x87, 0xe3, 0x48, 0x0b, 0x17, 0x0c, 0x13, 0xbc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EscReport__TYPE_NAME[] = "px4_msgs/msg/EscReport";

// Define type names, field names, and default values
static char px4_msgs__msg__EscReport__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_errorcount[] = "esc_errorcount";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_rpm[] = "esc_rpm";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_voltage[] = "esc_voltage";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_current[] = "esc_current";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_temperature[] = "esc_temperature";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_address[] = "esc_address";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_cmdcount[] = "esc_cmdcount";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_state[] = "esc_state";
static char px4_msgs__msg__EscReport__FIELD_NAME__actuator_function[] = "actuator_function";
static char px4_msgs__msg__EscReport__FIELD_NAME__failures[] = "failures";
static char px4_msgs__msg__EscReport__FIELD_NAME__esc_power[] = "esc_power";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EscReport__FIELDS[] = {
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_errorcount, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_rpm, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_voltage, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_current, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_temperature, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_address, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_cmdcount, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__actuator_function, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__failures, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EscReport__FIELD_NAME__esc_power, 9, 9},
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
px4_msgs__msg__EscReport__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EscReport__TYPE_NAME, 22, 22},
      {px4_msgs__msg__EscReport__FIELDS, 12, 12},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t\\t\\t# time since system start (microseconds)\n"
  "uint32 esc_errorcount\\t\\t\\t\\t\\t# Number of reported errors by ESC - if supported\n"
  "int32 esc_rpm\\t\\t\\t\\t\\t\\t# Motor RPM, negative for reverse rotation [RPM] - if supported\n"
  "float32 esc_voltage\\t\\t\\t\\t\\t# Voltage measured from current ESC [V] - if supported\n"
  "float32 esc_current\\t\\t\\t\\t\\t# Current measured from current ESC [A] - if supported\n"
  "float32 esc_temperature\\t\\t\\t\\t\\t# Temperature measured from current ESC [degC] - if supported\n"
  "uint8 esc_address\\t\\t\\t\\t\\t# Address of current ESC (in most cases 1-8 / must be set by driver)\n"
  "uint8 esc_cmdcount\\t\\t\\t\\t\\t# Counter of number of commands\n"
  "\n"
  "uint8 esc_state\\t\\t\\t\\t\\t# State of ESC - depend on Vendor\n"
  "\n"
  "uint8 actuator_function\\t\\t\\t\\t# actuator output function (one of Motor1...MotorN)\n"
  "\n"
  "uint16 failures\\t\\t\\t\\t\\t# Bitmask to indicate the internal ESC faults\n"
  "int8 esc_power\\t\\t\\t\\t\\t# Applied power 0-100 in % (negative values reserved)\n"
  "\n"
  "uint8 FAILURE_OVER_CURRENT = 0 \\t\\t\\t# (1 << 0)\n"
  "uint8 FAILURE_OVER_VOLTAGE = 1 \\t\\t\\t# (1 << 1)\n"
  "uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 2 \\t# (1 << 2)\n"
  "uint8 FAILURE_OVER_RPM = 3\\t\\t\\t# (1 << 3)\n"
  "uint8 FAILURE_INCONSISTENT_CMD = 4 \\t\\t# (1 << 4)  Set if ESC received an inconsistent command (i.e out of boundaries)\n"
  "uint8 FAILURE_MOTOR_STUCK = 5\\t\\t\\t# (1 << 5)\n"
  "uint8 FAILURE_GENERIC = 6\\t\\t\\t# (1 << 6)\n"
  "uint8 FAILURE_MOTOR_WARN_TEMPERATURE = 7\\t# (1 << 7)\n"
  "uint8 FAILURE_WARN_ESC_TEMPERATURE = 8\\t\\t# (1 << 8)\n"
  "uint8 FAILURE_OVER_ESC_TEMPERATURE = 9\\t\\t# (1 << 9)\n"
  "uint8 ESC_FAILURE_COUNT = 10 \\t\\t\\t# Counter - keep it as last element!";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EscReport__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EscReport__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1496, 1496},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EscReport__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EscReport__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
