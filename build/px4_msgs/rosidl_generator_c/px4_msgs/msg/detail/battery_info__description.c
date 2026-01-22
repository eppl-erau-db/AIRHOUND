// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/BatteryInfo.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/battery_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__BatteryInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbf, 0xb8, 0x01, 0xa5, 0x31, 0xb3, 0x0b, 0xba,
      0xe1, 0x2b, 0x29, 0x1e, 0xea, 0x73, 0xd8, 0x8a,
      0x72, 0x0e, 0xcf, 0xdd, 0xdb, 0xda, 0x6f, 0x88,
      0x4d, 0xd7, 0x81, 0xb5, 0x82, 0x0b, 0x3e, 0x37,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__BatteryInfo__TYPE_NAME[] = "px4_msgs/msg/BatteryInfo";

// Define type names, field names, and default values
static char px4_msgs__msg__BatteryInfo__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__BatteryInfo__FIELD_NAME__id[] = "id";
static char px4_msgs__msg__BatteryInfo__FIELD_NAME__serial_number[] = "serial_number";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__BatteryInfo__FIELDS[] = {
  {
    {px4_msgs__msg__BatteryInfo__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryInfo__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryInfo__FIELD_NAME__serial_number, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      32,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__BatteryInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__BatteryInfo__TYPE_NAME, 24, 24},
      {px4_msgs__msg__BatteryInfo__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Battery information\n"
  "#\n"
  "# Static or near-invariant battery information.\n"
  "# Should be streamed at low rate.\n"
  "\n"
  "uint64 timestamp # [us] Time since system start\n"
  "\n"
  "uint8 id # Must match the id in the battery_status message for the same battery\n"
  "char[32] serial_number # [@invalid 0 All bytes] Serial number of the battery pack in ASCII characters, 0 terminated";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__BatteryInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__BatteryInfo__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 352, 352},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__BatteryInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__BatteryInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
