// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/LandingTargetPose.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/landing_target_pose__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__LandingTargetPose__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe7, 0x64, 0xaa, 0xb0, 0x88, 0x72, 0x2a, 0x4b,
      0x9d, 0xff, 0x76, 0x00, 0x12, 0x3c, 0xce, 0x8a,
      0xd6, 0xdc, 0x19, 0x22, 0x49, 0xf0, 0xb5, 0x6e,
      0xd6, 0x58, 0x72, 0x61, 0x77, 0x17, 0x1c, 0x79,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__LandingTargetPose__TYPE_NAME[] = "px4_msgs/msg/LandingTargetPose";

// Define type names, field names, and default values
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__is_static[] = "is_static";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__rel_pos_valid[] = "rel_pos_valid";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__rel_vel_valid[] = "rel_vel_valid";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__x_rel[] = "x_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__y_rel[] = "y_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__z_rel[] = "z_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__vx_rel[] = "vx_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__vy_rel[] = "vy_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_x_rel[] = "cov_x_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_y_rel[] = "cov_y_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_vx_rel[] = "cov_vx_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_vy_rel[] = "cov_vy_rel";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__abs_pos_valid[] = "abs_pos_valid";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__x_abs[] = "x_abs";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__y_abs[] = "y_abs";
static char px4_msgs__msg__LandingTargetPose__FIELD_NAME__z_abs[] = "z_abs";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__LandingTargetPose__FIELDS[] = {
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__is_static, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__rel_pos_valid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__rel_vel_valid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__x_rel, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__y_rel, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__z_rel, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__vx_rel, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__vy_rel, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_x_rel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_y_rel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_vx_rel, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__cov_vy_rel, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__abs_pos_valid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__x_abs, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__y_abs, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__LandingTargetPose__FIELD_NAME__z_abs, 5, 5},
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
px4_msgs__msg__LandingTargetPose__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__LandingTargetPose__TYPE_NAME, 30, 30},
      {px4_msgs__msg__LandingTargetPose__FIELDS, 17, 17},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "bool is_static\\t\\t\\t# Flag indicating whether the landing target is static or moving with respect to the ground\n"
  "\n"
  "bool rel_pos_valid\\t\\t# Flag showing whether relative position is valid\n"
  "bool rel_vel_valid\\t\\t# Flag showing whether relative velocity is valid\n"
  "\n"
  "float32 x_rel\\t\\t\\t# X/north position of target, relative to vehicle (navigation frame) [meters]\n"
  "float32 y_rel\\t\\t\\t# Y/east position of target, relative to vehicle (navigation frame) [meters]\n"
  "float32 z_rel\\t\\t\\t# Z/down position of target, relative to vehicle (navigation frame) [meters]\n"
  "\n"
  "float32 vx_rel\\t\\t\\t# X/north velocity  of target, relative to vehicle (navigation frame) [meters/second]\n"
  "float32 vy_rel\\t\\t\\t# Y/east velocity of target, relative to vehicle (navigation frame) [meters/second]\n"
  "\n"
  "float32 cov_x_rel\\t\\t# X/north position variance [meters^2]\n"
  "float32 cov_y_rel\\t\\t# Y/east position variance [meters^2]\n"
  "\n"
  "float32 cov_vx_rel\\t\\t# X/north velocity variance [(meters/second)^2]\n"
  "float32 cov_vy_rel\\t\\t# Y/east velocity variance [(meters/second)^2]\n"
  "\n"
  "bool abs_pos_valid\\t\\t# Flag showing whether absolute position is valid\n"
  "float32 x_abs\\t\\t\\t# X/north position of target, relative to origin (navigation frame) [meters]\n"
  "float32 y_abs\\t\\t\\t# Y/east position of target, relative to origin (navigation frame) [meters]\n"
  "float32 z_abs\\t\\t\\t# Z/down position of target, relative to origin (navigation frame) [meters]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__LandingTargetPose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__LandingTargetPose__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1543, 1543},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__LandingTargetPose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__LandingTargetPose__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
