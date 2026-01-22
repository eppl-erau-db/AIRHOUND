// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FigureEightStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/figure_eight_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FigureEightStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0a, 0x6e, 0xc9, 0xfa, 0x70, 0x78, 0x00, 0xc3,
      0x25, 0x5b, 0xc2, 0xda, 0x22, 0x08, 0xc1, 0xd4,
      0x0e, 0x23, 0x1b, 0xbe, 0x04, 0x79, 0x8e, 0x25,
      0x9e, 0xa7, 0x74, 0x6f, 0x15, 0xdf, 0x69, 0xc6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FigureEightStatus__TYPE_NAME[] = "px4_msgs/msg/FigureEightStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__major_radius[] = "major_radius";
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__minor_radius[] = "minor_radius";
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__orientation[] = "orientation";
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__frame[] = "frame";
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__x[] = "x";
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__y[] = "y";
static char px4_msgs__msg__FigureEightStatus__FIELD_NAME__z[] = "z";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FigureEightStatus__FIELDS[] = {
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__major_radius, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__minor_radius, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__orientation, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__frame, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FigureEightStatus__FIELD_NAME__z, 1, 1},
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
px4_msgs__msg__FigureEightStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FigureEightStatus__TYPE_NAME, 30, 30},
      {px4_msgs__msg__FigureEightStatus__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "float32 major_radius \\t# Major axis radius of the figure eight [m]. Positive values orbit clockwise, negative values orbit counter-clockwise.\n"
  "float32 minor_radius \\t# Minor axis radius of the figure eight [m].\n"
  "float32 orientation \\t# Orientation of the major axis of the figure eight [rad].\n"
  "uint8 frame      # The coordinate system of the fields: x, y, z.\n"
  "int32 x          # X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.\n"
  "int32 y        \\t # Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters * 1e4, global = latitude in degrees * 1e7.\n"
  "float32 z        # Altitude of center point. Coordinate system depends on frame field.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FigureEightStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FigureEightStatus__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 824, 824},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FigureEightStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FigureEightStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
