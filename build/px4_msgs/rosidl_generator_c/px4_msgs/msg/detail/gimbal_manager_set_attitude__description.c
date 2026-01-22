// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GimbalManagerSetAttitude.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gimbal_manager_set_attitude__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GimbalManagerSetAttitude__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5a, 0x31, 0xf2, 0xe9, 0xeb, 0x72, 0xcb, 0x9c,
      0x79, 0x94, 0xde, 0x1d, 0xe0, 0xa4, 0xd2, 0xa3,
      0x32, 0xf4, 0x59, 0x22, 0x00, 0xda, 0xd7, 0x8b,
      0x23, 0x12, 0x2c, 0xf0, 0x32, 0xc1, 0xf9, 0x7e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GimbalManagerSetAttitude__TYPE_NAME[] = "px4_msgs/msg/GimbalManagerSetAttitude";

// Define type names, field names, and default values
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__origin_sysid[] = "origin_sysid";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__origin_compid[] = "origin_compid";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__target_system[] = "target_system";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__target_component[] = "target_component";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__flags[] = "flags";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__gimbal_device_id[] = "gimbal_device_id";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__q[] = "q";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__angular_velocity_x[] = "angular_velocity_x";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__angular_velocity_y[] = "angular_velocity_y";
static char px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__angular_velocity_z[] = "angular_velocity_z";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GimbalManagerSetAttitude__FIELDS[] = {
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__origin_sysid, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__origin_compid, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__target_system, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__target_component, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__flags, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__gimbal_device_id, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__angular_velocity_x, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__angular_velocity_y, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerSetAttitude__FIELD_NAME__angular_velocity_z, 18, 18},
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
px4_msgs__msg__GimbalManagerSetAttitude__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GimbalManagerSetAttitude__TYPE_NAME, 37, 37},
      {px4_msgs__msg__GimbalManagerSetAttitude__FIELDS, 11, 11},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 origin_sysid\n"
  "uint8 origin_compid\n"
  "\n"
  "uint8 target_system\n"
  "uint8 target_component\n"
  "\n"
  "uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1\n"
  "uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2\n"
  "uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4\n"
  "uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8\n"
  "uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16\n"
  "\n"
  "uint32 flags\n"
  "uint8 gimbal_device_id\n"
  "\n"
  "float32[4] q\n"
  "\n"
  "float32 angular_velocity_x\n"
  "float32 angular_velocity_y\n"
  "float32 angular_velocity_z\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GimbalManagerSetAttitude__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GimbalManagerSetAttitude__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 516, 516},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GimbalManagerSetAttitude__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GimbalManagerSetAttitude__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
