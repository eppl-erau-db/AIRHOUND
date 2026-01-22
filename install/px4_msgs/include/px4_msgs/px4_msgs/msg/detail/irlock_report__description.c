// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/IrlockReport.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/irlock_report__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__IrlockReport__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe9, 0x6b, 0xf1, 0x2f, 0x6c, 0x2c, 0x70, 0x3b,
      0x83, 0xb0, 0x27, 0x5c, 0x1d, 0xa6, 0xc3, 0xa8,
      0x01, 0x3c, 0x7a, 0xbf, 0x31, 0x5a, 0xb5, 0xe8,
      0xa9, 0x13, 0xe2, 0xd5, 0x8b, 0x08, 0x35, 0xdf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__IrlockReport__TYPE_NAME[] = "px4_msgs/msg/IrlockReport";

// Define type names, field names, and default values
static char px4_msgs__msg__IrlockReport__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__IrlockReport__FIELD_NAME__signature[] = "signature";
static char px4_msgs__msg__IrlockReport__FIELD_NAME__pos_x[] = "pos_x";
static char px4_msgs__msg__IrlockReport__FIELD_NAME__pos_y[] = "pos_y";
static char px4_msgs__msg__IrlockReport__FIELD_NAME__size_x[] = "size_x";
static char px4_msgs__msg__IrlockReport__FIELD_NAME__size_y[] = "size_y";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__IrlockReport__FIELDS[] = {
  {
    {px4_msgs__msg__IrlockReport__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__IrlockReport__FIELD_NAME__signature, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__IrlockReport__FIELD_NAME__pos_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__IrlockReport__FIELD_NAME__pos_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__IrlockReport__FIELD_NAME__size_x, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__IrlockReport__FIELD_NAME__size_y, 6, 6},
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
px4_msgs__msg__IrlockReport__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__IrlockReport__TYPE_NAME, 25, 25},
      {px4_msgs__msg__IrlockReport__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# IRLOCK_REPORT message data\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint16 signature\n"
  "\n"
  "# When looking along the optical axis of the camera, x points right, y points down, and z points along the optical axis.\n"
  "float32 pos_x # tan(theta), where theta is the angle between the target and the camera center of projection in camera x-axis\n"
  "float32 pos_y # tan(theta), where theta is the angle between the target and the camera center of projection in camera y-axis\n"
  "float32 size_x #/** size of target along camera x-axis in units of tan(theta) **/\n"
  "float32 size_y #/** size of target along camera y-axis in units of tan(theta) **/";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__IrlockReport__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__IrlockReport__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 643, 643},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__IrlockReport__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__IrlockReport__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
