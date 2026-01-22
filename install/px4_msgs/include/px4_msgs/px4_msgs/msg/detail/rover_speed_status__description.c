// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverSpeedStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_speed_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverSpeedStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x81, 0x9e, 0xcd, 0x32, 0x4c, 0xd9, 0x2a, 0x53,
      0xf1, 0x72, 0x10, 0x96, 0x1b, 0x9d, 0xb3, 0xe6,
      0xbe, 0xcb, 0x5b, 0x0d, 0xc0, 0x20, 0xeb, 0x9c,
      0x23, 0x84, 0x2a, 0x7a, 0xfc, 0xb8, 0x43, 0x9b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverSpeedStatus__TYPE_NAME[] = "px4_msgs/msg/RoverSpeedStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__measured_speed_body_x[] = "measured_speed_body_x";
static char px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__adjusted_speed_body_x_setpoint[] = "adjusted_speed_body_x_setpoint";
static char px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__pid_throttle_body_x_integral[] = "pid_throttle_body_x_integral";
static char px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__measured_speed_body_y[] = "measured_speed_body_y";
static char px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__adjusted_speed_body_y_setpoint[] = "adjusted_speed_body_y_setpoint";
static char px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__pid_throttle_body_y_integral[] = "pid_throttle_body_y_integral";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverSpeedStatus__FIELDS[] = {
  {
    {px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__measured_speed_body_x, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__adjusted_speed_body_x_setpoint, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__pid_throttle_body_x_integral, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__measured_speed_body_y, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__adjusted_speed_body_y_setpoint, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverSpeedStatus__FIELD_NAME__pid_throttle_body_y_integral, 28, 28},
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
px4_msgs__msg__RoverSpeedStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverSpeedStatus__TYPE_NAME, 29, 29},
      {px4_msgs__msg__RoverSpeedStatus__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rover Velocity Status\n"
  "\n"
  "uint64 timestamp                        # [us] Time since system start\n"
  "float32 measured_speed_body_x           # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Measured speed in body x direction\n"
  "float32 adjusted_speed_body_x_setpoint  # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction that is being tracked (Applied slew rates)\n"
  "float32 pid_throttle_body_x_integral    # [-] [@range -1, 1] Integral of the PID for the closed loop controller of the speed in body x direction\n"
  "float32 measured_speed_body_y           # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Measured speed in body y direction\n"
  "float32 adjusted_speed_body_y_setpoint  # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction that is being tracked (Applied slew rates)\n"
  "float32 pid_throttle_body_y_integral    # [-] [@range -1, 1] [@invalid NaN If not mecanum] Mecanum only: Integral of the PID for the closed loop controller of the speed in body y direction";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverSpeedStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverSpeedStatus__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1144, 1144},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverSpeedStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverSpeedStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
