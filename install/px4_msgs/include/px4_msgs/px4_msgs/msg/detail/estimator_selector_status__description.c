// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorSelectorStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_selector_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorSelectorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfc, 0x7c, 0x80, 0xf1, 0x84, 0x53, 0x8f, 0x36,
      0xd8, 0xed, 0x00, 0xac, 0x89, 0x36, 0x7d, 0x1e,
      0xa7, 0xcf, 0x62, 0xa2, 0xf2, 0x2c, 0xb2, 0x50,
      0xdd, 0xe3, 0x6f, 0x78, 0xf3, 0x01, 0xb0, 0x03,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorSelectorStatus__TYPE_NAME[] = "px4_msgs/msg/EstimatorSelectorStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__primary_instance[] = "primary_instance";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__instances_available[] = "instances_available";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__instance_changed_count[] = "instance_changed_count";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__last_instance_change[] = "last_instance_change";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accel_device_id[] = "accel_device_id";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__baro_device_id[] = "baro_device_id";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__gyro_device_id[] = "gyro_device_id";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__mag_device_id[] = "mag_device_id";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__combined_test_ratio[] = "combined_test_ratio";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__relative_test_ratio[] = "relative_test_ratio";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__healthy[] = "healthy";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accumulated_gyro_error[] = "accumulated_gyro_error";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accumulated_accel_error[] = "accumulated_accel_error";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__gyro_fault_detected[] = "gyro_fault_detected";
static char px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accel_fault_detected[] = "accel_fault_detected";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorSelectorStatus__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__primary_instance, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__instances_available, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__instance_changed_count, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__last_instance_change, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accel_device_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__baro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__gyro_device_id, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__mag_device_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__combined_test_ratio, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__relative_test_ratio, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__healthy, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accumulated_gyro_error, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accumulated_accel_error, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__gyro_fault_detected, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorSelectorStatus__FIELD_NAME__accel_fault_detected, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__EstimatorSelectorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorSelectorStatus__TYPE_NAME, 36, 36},
      {px4_msgs__msg__EstimatorSelectorStatus__FIELDS, 16, 16},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 primary_instance\n"
  "\n"
  "uint8 instances_available\n"
  "\n"
  "uint32 instance_changed_count\n"
  "uint64 last_instance_change\n"
  "\n"
  "uint32 accel_device_id\n"
  "uint32 baro_device_id\n"
  "uint32 gyro_device_id\n"
  "uint32 mag_device_id\n"
  "\n"
  "float32[9] combined_test_ratio\n"
  "float32[9] relative_test_ratio\n"
  "bool[9] healthy\n"
  "\n"
  "float32[4] accumulated_gyro_error\n"
  "float32[4] accumulated_accel_error\n"
  "bool gyro_fault_detected\n"
  "bool accel_fault_detected";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorSelectorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorSelectorStatus__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 458, 458},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorSelectorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorSelectorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
