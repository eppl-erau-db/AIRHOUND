// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/DatamanRequest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/dataman_request__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__DatamanRequest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xca, 0x33, 0xb6, 0x3f, 0x08, 0x9b, 0xbc, 0xa6,
      0x65, 0xf7, 0xcd, 0xc3, 0x8b, 0x95, 0xd3, 0x45,
      0x1b, 0x88, 0xe7, 0xba, 0xd0, 0xb4, 0xf5, 0x2b,
      0x94, 0x1b, 0x66, 0x2e, 0xe2, 0x55, 0xa9, 0x75,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__DatamanRequest__TYPE_NAME[] = "px4_msgs/msg/DatamanRequest";

// Define type names, field names, and default values
static char px4_msgs__msg__DatamanRequest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__DatamanRequest__FIELD_NAME__client_id[] = "client_id";
static char px4_msgs__msg__DatamanRequest__FIELD_NAME__request_type[] = "request_type";
static char px4_msgs__msg__DatamanRequest__FIELD_NAME__item[] = "item";
static char px4_msgs__msg__DatamanRequest__FIELD_NAME__index[] = "index";
static char px4_msgs__msg__DatamanRequest__FIELD_NAME__data[] = "data";
static char px4_msgs__msg__DatamanRequest__FIELD_NAME__data_length[] = "data_length";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__DatamanRequest__FIELDS[] = {
  {
    {px4_msgs__msg__DatamanRequest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DatamanRequest__FIELD_NAME__client_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DatamanRequest__FIELD_NAME__request_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DatamanRequest__FIELD_NAME__item, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DatamanRequest__FIELD_NAME__index, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DatamanRequest__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      56,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DatamanRequest__FIELD_NAME__data_length, 11, 11},
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
px4_msgs__msg__DatamanRequest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__DatamanRequest__TYPE_NAME, 27, 27},
      {px4_msgs__msg__DatamanRequest__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 client_id\n"
  "uint8 request_type\\t# id/read/write/clear\n"
  "uint8 item\\t\\t\\t# dm_item_t\n"
  "uint32 index\n"
  "uint8[56] data\n"
  "uint32 data_length";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__DatamanRequest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__DatamanRequest__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 188, 188},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__DatamanRequest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__DatamanRequest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
