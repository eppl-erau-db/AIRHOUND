// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GotoSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/goto_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GotoSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xef, 0x99, 0xfb, 0x91, 0xae, 0x3a, 0x8c, 0xc2,
      0xae, 0x20, 0x52, 0xaa, 0xd4, 0x18, 0xe2, 0xbd,
      0x2d, 0x8a, 0xcf, 0xa4, 0x9b, 0x0f, 0x25, 0xcf,
      0xba, 0x78, 0xf5, 0x59, 0x39, 0xb4, 0x83, 0x5b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GotoSetpoint__TYPE_NAME[] = "px4_msgs/msg/GotoSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__position[] = "position";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_control_heading[] = "flag_control_heading";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__heading[] = "heading";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_set_max_horizontal_speed[] = "flag_set_max_horizontal_speed";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__max_horizontal_speed[] = "max_horizontal_speed";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_set_max_vertical_speed[] = "flag_set_max_vertical_speed";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__max_vertical_speed[] = "max_vertical_speed";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_set_max_heading_rate[] = "flag_set_max_heading_rate";
static char px4_msgs__msg__GotoSetpoint__FIELD_NAME__max_heading_rate[] = "max_heading_rate";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GotoSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_control_heading, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_set_max_horizontal_speed, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__max_horizontal_speed, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_set_max_vertical_speed, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__max_vertical_speed, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__flag_set_max_heading_rate, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GotoSetpoint__FIELD_NAME__max_heading_rate, 16, 16},
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
px4_msgs__msg__GotoSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GotoSetpoint__TYPE_NAME, 25, 25},
      {px4_msgs__msg__GotoSetpoint__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Position and (optional) heading setpoints with corresponding speed constraints\n"
  "# Setpoints are intended as inputs to position and heading smoothers, respectively\n"
  "# Setpoints do not need to be kinematically consistent\n"
  "# Optional heading setpoints may be specified as controlled by the respective flag\n"
  "# Unset optional setpoints are not controlled\n"
  "# Unset optional constraints default to vehicle specifications\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "# setpoints\n"
  "float32[3] position # [m] NED local world frame\n"
  "\n"
  "bool flag_control_heading # true if heading is to be controlled\n"
  "float32 heading # (optional) [rad] [-pi,pi] from North\n"
  "\n"
  "# constraints\n"
  "bool flag_set_max_horizontal_speed # true if setting a non-default horizontal speed limit\n"
  "float32 max_horizontal_speed # (optional) [m/s] maximum speed (absolute) in the NE-plane\n"
  "\n"
  "bool flag_set_max_vertical_speed # true if setting a non-default vertical speed limit\n"
  "float32 max_vertical_speed # (optional) [m/s] maximum speed (absolute) in the D-axis\n"
  "\n"
  "bool flag_set_max_heading_rate # true if setting a non-default heading rate limit\n"
  "float32 max_heading_rate # (optional) [rad/s] maximum heading rate (absolute)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GotoSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GotoSetpoint__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1206, 1206},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GotoSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GotoSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
