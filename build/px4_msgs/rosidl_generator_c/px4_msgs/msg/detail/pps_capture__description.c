// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/PpsCapture.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/pps_capture__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__PpsCapture__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4b, 0xff, 0x24, 0x78, 0xe0, 0xd1, 0x13, 0xb9,
      0x17, 0x73, 0xfa, 0x3a, 0xf1, 0xd7, 0x62, 0xdc,
      0x8f, 0xf5, 0xca, 0x52, 0x5b, 0x1a, 0xe6, 0x21,
      0x0e, 0x5a, 0x86, 0x02, 0x35, 0x83, 0xbf, 0xbe,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__PpsCapture__TYPE_NAME[] = "px4_msgs/msg/PpsCapture";

// Define type names, field names, and default values
static char px4_msgs__msg__PpsCapture__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__PpsCapture__FIELD_NAME__rtc_timestamp[] = "rtc_timestamp";
static char px4_msgs__msg__PpsCapture__FIELD_NAME__pps_rate_exceeded_counter[] = "pps_rate_exceeded_counter";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__PpsCapture__FIELDS[] = {
  {
    {px4_msgs__msg__PpsCapture__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PpsCapture__FIELD_NAME__rtc_timestamp, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PpsCapture__FIELD_NAME__pps_rate_exceeded_counter, 25, 25},
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
px4_msgs__msg__PpsCapture__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__PpsCapture__TYPE_NAME, 23, 23},
      {px4_msgs__msg__PpsCapture__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t  # time since system start (microseconds) at PPS capture event\n"
  "uint64 rtc_timestamp\\t\\t# Corrected GPS UTC timestamp at PPS capture event\n"
  "uint8  pps_rate_exceeded_counter # Increments when PPS dt < 50ms";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__PpsCapture__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__PpsCapture__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 221, 221},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__PpsCapture__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__PpsCapture__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
