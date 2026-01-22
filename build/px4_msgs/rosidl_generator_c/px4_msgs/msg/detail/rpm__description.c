// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Rpm.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rpm__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Rpm__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x31, 0xd1, 0xaf, 0x67, 0xe5, 0xe0, 0x95, 0x0a,
      0x9c, 0xfe, 0xdb, 0x1d, 0x95, 0xf5, 0x85, 0x46,
      0xae, 0x6c, 0x75, 0x60, 0x49, 0xf1, 0xfd, 0x10,
      0x8e, 0xd4, 0xfd, 0x33, 0xde, 0x4f, 0x35, 0x06,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Rpm__TYPE_NAME[] = "px4_msgs/msg/Rpm";

// Define type names, field names, and default values
static char px4_msgs__msg__Rpm__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Rpm__FIELD_NAME__rpm_estimate[] = "rpm_estimate";
static char px4_msgs__msg__Rpm__FIELD_NAME__rpm_raw[] = "rpm_raw";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Rpm__FIELDS[] = {
  {
    {px4_msgs__msg__Rpm__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Rpm__FIELD_NAME__rpm_estimate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Rpm__FIELD_NAME__rpm_raw, 7, 7},
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
px4_msgs__msg__Rpm__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Rpm__TYPE_NAME, 16, 16},
      {px4_msgs__msg__Rpm__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "# rpm values of 0.0 mean within a timeout there is no movement measured\n"
  "float32 rpm_estimate # filtered revolutions per minute\n"
  "float32 rpm_raw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Rpm__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Rpm__TYPE_NAME, 16, 16},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 202, 202},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Rpm__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Rpm__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
