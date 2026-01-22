// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ManualControlSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/manual_control_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ManualControlSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2e, 0xc0, 0xe5, 0x28, 0xa3, 0x55, 0x53, 0x3a,
      0xd4, 0xc6, 0x68, 0x54, 0xef, 0xef, 0x10, 0x4b,
      0x62, 0xf4, 0x2e, 0xde, 0xd5, 0xd8, 0x30, 0xaf,
      0x9d, 0xc1, 0x8a, 0xb2, 0x05, 0xb8, 0x25, 0xed,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ManualControlSetpoint__TYPE_NAME[] = "px4_msgs/msg/ManualControlSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__valid[] = "valid";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__data_source[] = "data_source";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__roll[] = "roll";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__pitch[] = "pitch";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__yaw[] = "yaw";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__throttle[] = "throttle";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__flaps[] = "flaps";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux1[] = "aux1";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux2[] = "aux2";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux3[] = "aux3";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux4[] = "aux4";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux5[] = "aux5";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux6[] = "aux6";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__sticks_moving[] = "sticks_moving";
static char px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__buttons[] = "buttons";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ManualControlSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__valid, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__data_source, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__throttle, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__flaps, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux1, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux2, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux3, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux4, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux5, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__aux6, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__sticks_moving, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSetpoint__FIELD_NAME__buttons, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ManualControlSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ManualControlSetpoint__TYPE_NAME, 34, 34},
      {px4_msgs__msg__ManualControlSetpoint__FIELDS, 17, 17},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp                        # time since system start (microseconds)\n"
  "uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)\n"
  "\n"
  "bool valid\n"
  "\n"
  "uint8 SOURCE_UNKNOWN   = 0\n"
  "uint8 SOURCE_RC        = 1\\t\\t# radio control (input_rc)\n"
  "uint8 SOURCE_MAVLINK_0 = 2\\t\\t# mavlink instance 0\n"
  "uint8 SOURCE_MAVLINK_1 = 3\\t\\t# mavlink instance 1\n"
  "uint8 SOURCE_MAVLINK_2 = 4\\t\\t# mavlink instance 2\n"
  "uint8 SOURCE_MAVLINK_3 = 5\\t\\t# mavlink instance 3\n"
  "uint8 SOURCE_MAVLINK_4 = 6\\t\\t# mavlink instance 4\n"
  "uint8 SOURCE_MAVLINK_5 = 7\\t\\t# mavlink instance 5\n"
  "\n"
  "uint8 data_source\n"
  "\n"
  "# Any of the channels may not be available and be set to NaN\n"
  "# to indicate that it does not contain valid data.\n"
  "\n"
  "# Stick positions [-1,1]\n"
  "# on a common RC mode 1/2/3/4 remote/joystick the stick deflection: -1 is down/left, 1 is up/right\n"
  "# Note: QGC sends throttle/z in range [0,1000] - [0,1]. The MAVLink input conversion [0,1] to [-1,1] is at the moment kept backwards compatible.\n"
  "# Positive values are generally used for:\n"
  "float32 roll     # move right,   positive roll rotation,  right side down\n"
  "float32 pitch    # move forward, negative pitch rotation, nose down\n"
  "float32 yaw      #               positive yaw rotation,   clockwise when seen top down\n"
  "float32 throttle # move up,      positive thrust,         -1 is minimum available 0% or -100% +1 is 100% thrust\n"
  "\n"
  "float32 flaps\\t\\t\\t # position of flaps switch/knob/lever [-1, 1]\n"
  "\n"
  "float32 aux1\n"
  "float32 aux2\n"
  "float32 aux3\n"
  "float32 aux4\n"
  "float32 aux5\n"
  "float32 aux6\n"
  "\n"
  "bool sticks_moving\n"
  "\n"
  "uint16 buttons\\t\\t# From uint16 buttons field of Mavlink manual_control message\n"
  "\n"
  "# TOPICS manual_control_setpoint manual_control_input\n"
  "# DEPRECATED: float32 x\n"
  "# DEPRECATED: float32 y\n"
  "# DEPRECATED: float32 z\n"
  "# DEPRECATED: float32 r";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ManualControlSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ManualControlSetpoint__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1763, 1763},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ManualControlSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ManualControlSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
