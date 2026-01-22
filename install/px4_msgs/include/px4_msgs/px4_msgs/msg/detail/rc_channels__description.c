// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RcChannels.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rc_channels__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RcChannels__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8c, 0x00, 0x48, 0xdf, 0xc1, 0x2b, 0x5f, 0x25,
      0xd6, 0xe6, 0x71, 0x86, 0x18, 0x94, 0xe6, 0x8b,
      0xd4, 0x33, 0x90, 0x06, 0x8e, 0x0d, 0x1d, 0x33,
      0x11, 0x97, 0x47, 0xce, 0xb7, 0xb9, 0x53, 0xde,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RcChannels__TYPE_NAME[] = "px4_msgs/msg/RcChannels";

// Define type names, field names, and default values
static char px4_msgs__msg__RcChannels__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RcChannels__FIELD_NAME__timestamp_last_valid[] = "timestamp_last_valid";
static char px4_msgs__msg__RcChannels__FIELD_NAME__channels[] = "channels";
static char px4_msgs__msg__RcChannels__FIELD_NAME__channel_count[] = "channel_count";
static char px4_msgs__msg__RcChannels__FIELD_NAME__function[] = "function";
static char px4_msgs__msg__RcChannels__FIELD_NAME__rssi[] = "rssi";
static char px4_msgs__msg__RcChannels__FIELD_NAME__signal_lost[] = "signal_lost";
static char px4_msgs__msg__RcChannels__FIELD_NAME__frame_drop_count[] = "frame_drop_count";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RcChannels__FIELDS[] = {
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__timestamp_last_valid, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__channels, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      18,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__channel_count, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__function, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8_ARRAY,
      30,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__rssi, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__signal_lost, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RcChannels__FIELD_NAME__frame_drop_count, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__RcChannels__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RcChannels__TYPE_NAME, 23, 23},
      {px4_msgs__msg__RcChannels__FIELDS, 8, 8},
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
  "uint8 FUNCTION_THROTTLE      = 0\n"
  "uint8 FUNCTION_ROLL          = 1\n"
  "uint8 FUNCTION_PITCH         = 2\n"
  "uint8 FUNCTION_YAW           = 3\n"
  "uint8 FUNCTION_RETURN        = 4\n"
  "uint8 FUNCTION_LOITER        = 5\n"
  "uint8 FUNCTION_OFFBOARD      = 6\n"
  "uint8 FUNCTION_FLAPS         = 7\n"
  "uint8 FUNCTION_AUX_1         = 8\n"
  "uint8 FUNCTION_AUX_2         = 9\n"
  "uint8 FUNCTION_AUX_3         = 10\n"
  "uint8 FUNCTION_AUX_4         = 11\n"
  "uint8 FUNCTION_AUX_5         = 12\n"
  "uint8 FUNCTION_AUX_6         = 13\n"
  "uint8 FUNCTION_PARAM_1       = 14\n"
  "uint8 FUNCTION_PARAM_2       = 15\n"
  "uint8 FUNCTION_PARAM_3_5     = 16\n"
  "uint8 FUNCTION_KILLSWITCH    = 17\n"
  "uint8 FUNCTION_TRANSITION    = 18\n"
  "uint8 FUNCTION_GEAR          = 19\n"
  "uint8 FUNCTION_ARMSWITCH     = 20\n"
  "uint8 FUNCTION_FLTBTN_SLOT_1 = 21\n"
  "uint8 FUNCTION_FLTBTN_SLOT_2 = 22\n"
  "uint8 FUNCTION_FLTBTN_SLOT_3 = 23\n"
  "uint8 FUNCTION_FLTBTN_SLOT_4 = 24\n"
  "uint8 FUNCTION_FLTBTN_SLOT_5 = 25\n"
  "uint8 FUNCTION_FLTBTN_SLOT_6 = 26\n"
  "uint8 FUNCTION_ENGAGE_MAIN_MOTOR = 27\n"
  "uint8 FUNCTION_PAYLOAD_POWER = 28\n"
  "uint8 FUNCTION_TERMINATION = 29\n"
  "\n"
  "uint8 FUNCTION_FLTBTN_SLOT_COUNT = 6\n"
  "\n"
  "uint64 timestamp_last_valid\\t\\t\\t\\t\\t# Timestamp of last valid RC signal\n"
  "float32[18] channels\\t\\t\\t\\t\\t\\t# Scaled to -1..1 (throttle: 0..1)\n"
  "uint8 channel_count\\t\\t\\t\\t\\t\\t# Number of valid channels\n"
  "int8[30] function\\t\\t\\t\\t\\t\\t# Functions mapping\n"
  "uint8 rssi\\t\\t\\t\\t\\t\\t\\t# Receive signal strength index\n"
  "bool signal_lost\\t\\t\\t\\t\\t\\t# Control signal lost, should be checked together with topic timeout\n"
  "uint32 frame_drop_count\\t\\t\\t\\t\\t\\t# Number of dropped frames";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RcChannels__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RcChannels__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1535, 1535},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RcChannels__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RcChannels__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
