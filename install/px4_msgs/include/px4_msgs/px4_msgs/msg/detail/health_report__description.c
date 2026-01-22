// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/HealthReport.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/health_report__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__HealthReport__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x32, 0xe8, 0x63, 0xf7, 0x4b, 0x00, 0x94, 0x6c,
      0x77, 0x33, 0xca, 0x7b, 0x7e, 0x3b, 0x25, 0xcf,
      0x6b, 0xa4, 0x65, 0x51, 0xc7, 0xad, 0xe3, 0xb8,
      0x82, 0xbc, 0x14, 0x10, 0xd7, 0x44, 0x0c, 0x09,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__HealthReport__TYPE_NAME[] = "px4_msgs/msg/HealthReport";

// Define type names, field names, and default values
static char px4_msgs__msg__HealthReport__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__HealthReport__FIELD_NAME__can_arm_mode_flags[] = "can_arm_mode_flags";
static char px4_msgs__msg__HealthReport__FIELD_NAME__can_run_mode_flags[] = "can_run_mode_flags";
static char px4_msgs__msg__HealthReport__FIELD_NAME__health_is_present_flags[] = "health_is_present_flags";
static char px4_msgs__msg__HealthReport__FIELD_NAME__health_warning_flags[] = "health_warning_flags";
static char px4_msgs__msg__HealthReport__FIELD_NAME__health_error_flags[] = "health_error_flags";
static char px4_msgs__msg__HealthReport__FIELD_NAME__arming_check_warning_flags[] = "arming_check_warning_flags";
static char px4_msgs__msg__HealthReport__FIELD_NAME__arming_check_error_flags[] = "arming_check_error_flags";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__HealthReport__FIELDS[] = {
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__can_arm_mode_flags, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__can_run_mode_flags, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__health_is_present_flags, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__health_warning_flags, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__health_error_flags, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__arming_check_warning_flags, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HealthReport__FIELD_NAME__arming_check_error_flags, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__HealthReport__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__HealthReport__TYPE_NAME, 25, 25},
      {px4_msgs__msg__HealthReport__FIELDS, 8, 8},
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
  "uint64 can_arm_mode_flags              # bitfield for each flight mode (NAVIGATION_STATE_*) if arming is possible\n"
  "uint64 can_run_mode_flags              # bitfield for each flight mode if it can run\n"
  "\n"
  "uint64 health_is_present_flags         # flags for each health_component_t\n"
  "uint64 health_warning_flags\n"
  "uint64 health_error_flags\n"
  "# A component is required but missing, if present==0 and error==1\n"
  "\n"
  "uint64 arming_check_warning_flags\n"
  "uint64 arming_check_error_flags";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__HealthReport__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__HealthReport__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 521, 521},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__HealthReport__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__HealthReport__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
