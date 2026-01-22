// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GimbalManagerInformation.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gimbal_manager_information__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GimbalManagerInformation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb2, 0xb5, 0x18, 0x2f, 0xc5, 0xe7, 0x3a, 0x46,
      0x4a, 0x04, 0xea, 0xbe, 0x4d, 0x51, 0x10, 0x45,
      0x01, 0xec, 0xc7, 0xdf, 0x61, 0x5e, 0xe9, 0x59,
      0x97, 0x31, 0xd9, 0xed, 0xea, 0xc7, 0x03, 0x5b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GimbalManagerInformation__TYPE_NAME[] = "px4_msgs/msg/GimbalManagerInformation";

// Define type names, field names, and default values
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__cap_flags[] = "cap_flags";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__gimbal_device_id[] = "gimbal_device_id";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__roll_min[] = "roll_min";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__roll_max[] = "roll_max";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__pitch_min[] = "pitch_min";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__pitch_max[] = "pitch_max";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__yaw_min[] = "yaw_min";
static char px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__yaw_max[] = "yaw_max";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GimbalManagerInformation__FIELDS[] = {
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__cap_flags, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__gimbal_device_id, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__roll_min, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__roll_max, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__pitch_min, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__pitch_max, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__yaw_min, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GimbalManagerInformation__FIELD_NAME__yaw_max, 7, 7},
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
px4_msgs__msg__GimbalManagerInformation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GimbalManagerInformation__TYPE_NAME, 37, 37},
      {px4_msgs__msg__GimbalManagerInformation__FIELDS, 9, 9},
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
  "uint32 cap_flags\n"
  "\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536\n"
  "uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072\n"
  "\n"
  "uint8 gimbal_device_id\n"
  "\n"
  "float32 roll_min # [rad]\n"
  "float32 roll_max # [rad]\n"
  "\n"
  "float32 pitch_min # [rad]\n"
  "float32 pitch_max # [rad]\n"
  "\n"
  "float32 yaw_min # [rad]\n"
  "float32 yaw_max # [rad]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GimbalManagerInformation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GimbalManagerInformation__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1016, 1016},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GimbalManagerInformation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GimbalManagerInformation__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
