// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/GpsDump.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/gps_dump__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__GpsDump__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfa, 0x28, 0x35, 0x29, 0xd7, 0xfa, 0x01, 0x91,
      0xb8, 0x4e, 0xcb, 0xb4, 0x8d, 0x20, 0xe8, 0x50,
      0x92, 0xe0, 0x60, 0x95, 0x9e, 0xd4, 0xbe, 0x61,
      0x0d, 0x3e, 0xd2, 0xbc, 0xfd, 0xeb, 0x04, 0x55,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__GpsDump__TYPE_NAME[] = "px4_msgs/msg/GpsDump";

// Define type names, field names, and default values
static char px4_msgs__msg__GpsDump__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__GpsDump__FIELD_NAME__instance[] = "instance";
static char px4_msgs__msg__GpsDump__FIELD_NAME__len[] = "len";
static char px4_msgs__msg__GpsDump__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__GpsDump__FIELDS[] = {
  {
    {px4_msgs__msg__GpsDump__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpsDump__FIELD_NAME__instance, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpsDump__FIELD_NAME__len, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__GpsDump__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      79,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__GpsDump__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__GpsDump__TYPE_NAME, 20, 20},
      {px4_msgs__msg__GpsDump__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message is used to dump the raw gps communication to the log.\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "uint8 instance   # Instance of GNSS receiver\n"
  "uint8 len        # length of data, MSB bit set = message to the gps device,\n"
  "                 # clear = message from the device\n"
  "uint8[79] data   # data to write to the log\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 8";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__GpsDump__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__GpsDump__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 373, 373},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__GpsDump__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__GpsDump__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
