// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OrbitStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/orbit_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OrbitStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4a, 0xc9, 0xd5, 0x59, 0xf5, 0x06, 0x59, 0x97,
      0x53, 0x00, 0xa5, 0x3c, 0xb9, 0xf2, 0x42, 0x0f,
      0x63, 0x10, 0x0d, 0x89, 0x47, 0xff, 0x8c, 0x8b,
      0xb2, 0xb7, 0xff, 0x55, 0x0b, 0x1f, 0x9d, 0x73,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OrbitStatus__TYPE_NAME[] = "px4_msgs/msg/OrbitStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__OrbitStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OrbitStatus__FIELD_NAME__radius[] = "radius";
static char px4_msgs__msg__OrbitStatus__FIELD_NAME__frame[] = "frame";
static char px4_msgs__msg__OrbitStatus__FIELD_NAME__x[] = "x";
static char px4_msgs__msg__OrbitStatus__FIELD_NAME__y[] = "y";
static char px4_msgs__msg__OrbitStatus__FIELD_NAME__z[] = "z";
static char px4_msgs__msg__OrbitStatus__FIELD_NAME__yaw_behaviour[] = "yaw_behaviour";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OrbitStatus__FIELDS[] = {
  {
    {px4_msgs__msg__OrbitStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbitStatus__FIELD_NAME__radius, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbitStatus__FIELD_NAME__frame, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbitStatus__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbitStatus__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbitStatus__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OrbitStatus__FIELD_NAME__yaw_behaviour, 13, 13},
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
px4_msgs__msg__OrbitStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OrbitStatus__TYPE_NAME, 24, 24},
      {px4_msgs__msg__OrbitStatus__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ORBIT_YAW_BEHAVIOUR\n"
  "uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0\n"
  "uint8 ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1\n"
  "uint8 ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2\n"
  "uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3\n"
  "uint8 ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4\n"
  "uint8 ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "float32 radius   # Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise. [m]\n"
  "uint8 frame      # The coordinate system of the fields: x, y, z.\n"
  "float64 x        # X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.\n"
  "float64 y        # Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters * 1e4, global = latitude in degrees * 1e7.\n"
  "float32 z        # Altitude of center point. Coordinate system depends on frame field.\n"
  "uint8 yaw_behaviour";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OrbitStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OrbitStatus__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 999, 999},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OrbitStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OrbitStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
