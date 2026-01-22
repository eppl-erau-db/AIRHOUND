// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/InputRc.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/input_rc__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__InputRc__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5c, 0xb6, 0x72, 0xe6, 0x58, 0x94, 0x95, 0x09,
      0xf2, 0x26, 0x47, 0x57, 0x3c, 0x13, 0x13, 0x9b,
      0xb0, 0xc2, 0x2d, 0x70, 0x43, 0xce, 0xa4, 0x16,
      0xe0, 0xad, 0x71, 0x60, 0x6a, 0xe3, 0x1d, 0xac,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__InputRc__TYPE_NAME[] = "px4_msgs/msg/InputRc";

// Define type names, field names, and default values
static char px4_msgs__msg__InputRc__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__InputRc__FIELD_NAME__timestamp_last_signal[] = "timestamp_last_signal";
static char px4_msgs__msg__InputRc__FIELD_NAME__channel_count[] = "channel_count";
static char px4_msgs__msg__InputRc__FIELD_NAME__rssi[] = "rssi";
static char px4_msgs__msg__InputRc__FIELD_NAME__rc_failsafe[] = "rc_failsafe";
static char px4_msgs__msg__InputRc__FIELD_NAME__rc_lost[] = "rc_lost";
static char px4_msgs__msg__InputRc__FIELD_NAME__rc_lost_frame_count[] = "rc_lost_frame_count";
static char px4_msgs__msg__InputRc__FIELD_NAME__rc_total_frame_count[] = "rc_total_frame_count";
static char px4_msgs__msg__InputRc__FIELD_NAME__rc_ppm_frame_length[] = "rc_ppm_frame_length";
static char px4_msgs__msg__InputRc__FIELD_NAME__input_source[] = "input_source";
static char px4_msgs__msg__InputRc__FIELD_NAME__values[] = "values";
static char px4_msgs__msg__InputRc__FIELD_NAME__link_quality[] = "link_quality";
static char px4_msgs__msg__InputRc__FIELD_NAME__rssi_dbm[] = "rssi_dbm";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__InputRc__FIELDS[] = {
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__timestamp_last_signal, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__channel_count, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__rssi, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__rc_failsafe, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__rc_lost, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__rc_lost_frame_count, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__rc_total_frame_count, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__rc_ppm_frame_length, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__input_source, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__values, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      18,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__link_quality, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__InputRc__FIELD_NAME__rssi_dbm, 8, 8},
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
px4_msgs__msg__InputRc__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__InputRc__TYPE_NAME, 20, 20},
      {px4_msgs__msg__InputRc__FIELDS, 13, 13},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 RC_INPUT_SOURCE_UNKNOWN = 0\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_PPM = 1\n"
  "uint8 RC_INPUT_SOURCE_PX4IO_PPM = 2\n"
  "uint8 RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3\n"
  "uint8 RC_INPUT_SOURCE_PX4IO_SBUS = 4\n"
  "uint8 RC_INPUT_SOURCE_PX4IO_ST24 = 5\n"
  "uint8 RC_INPUT_SOURCE_MAVLINK = 6\n"
  "uint8 RC_INPUT_SOURCE_QURT = 7\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_SBUS = 9\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_ST24 = 10\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_SUMD = 11\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_DSM = 12\n"
  "uint8 RC_INPUT_SOURCE_PX4IO_SUMD = 13\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_CRSF = 14\n"
  "uint8 RC_INPUT_SOURCE_PX4FMU_GHST = 15\n"
  "\n"
  "uint8 RC_INPUT_MAX_CHANNELS = 18 \\t# Maximum number of R/C input channels in the system. S.Bus has up to 18 channels.\n"
  "\n"
  "uint64 timestamp_last_signal\\t\\t# last valid reception time\n"
  "\n"
  "uint8 channel_count\\t\\t\\t# number of channels actually being seen\n"
  "\n"
  "int8 RSSI_MAX = 100\n"
  "int32 rssi\\t\\t\\t\\t# receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 100: full reception\n"
  "\n"
  "bool rc_failsafe\\t\\t\\t# explicit failsafe flag: true on TX failure or TX out of range , false otherwise. Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.\n"
  "bool rc_lost\\t\\t\\t\\t# RC receiver connection status: True,if no frame has arrived in the expected time, false otherwise. True usually means that the receiver has been disconnected, but can also indicate a radio link loss on \"stupid\" systems. Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.\n"
  "\n"
  "uint16 rc_lost_frame_count\\t\\t# Number of lost RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.\n"
  "uint16 rc_total_frame_count\\t\\t# Number of total RC frames. Note: intended purpose: observe the radio link quality if RSSI is not available. This value must not be used to trigger any failsafe-alike functionality.\n"
  "uint16 rc_ppm_frame_length\\t\\t# Length of a single PPM frame. Zero for non-PPM systems\n"
  "\n"
  "uint8 input_source\\t\\t\\t# Input source\n"
  "uint16[18] values\\t\\t\\t# measured pulse widths for each of the supported channels\n"
  "\n"
  "int8 link_quality\\t\\t\\t# link quality. Percentage 0-100%. -1 = invalid\n"
  "float32 rssi_dbm\\t\\t\\t# Actual rssi in units of dBm. NaN = invalid";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__InputRc__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__InputRc__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2356, 2356},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__InputRc__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__InputRc__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
