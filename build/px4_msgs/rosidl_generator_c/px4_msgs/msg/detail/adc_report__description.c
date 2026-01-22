// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/AdcReport.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/adc_report__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__AdcReport__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x08, 0x9e, 0xc5, 0x48, 0x29, 0xec, 0xa6, 0x50,
      0x86, 0xdc, 0x84, 0xcd, 0x8a, 0x03, 0x71, 0x02,
      0x43, 0xf1, 0x2d, 0x7d, 0x94, 0xc3, 0x4a, 0xdf,
      0x57, 0x20, 0xa0, 0x94, 0x5f, 0xa3, 0xd1, 0x03,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__AdcReport__TYPE_NAME[] = "px4_msgs/msg/AdcReport";

// Define type names, field names, and default values
static char px4_msgs__msg__AdcReport__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__AdcReport__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__AdcReport__FIELD_NAME__channel_id[] = "channel_id";
static char px4_msgs__msg__AdcReport__FIELD_NAME__raw_data[] = "raw_data";
static char px4_msgs__msg__AdcReport__FIELD_NAME__resolution[] = "resolution";
static char px4_msgs__msg__AdcReport__FIELD_NAME__v_ref[] = "v_ref";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__AdcReport__FIELDS[] = {
  {
    {px4_msgs__msg__AdcReport__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AdcReport__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AdcReport__FIELD_NAME__channel_id, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      12,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AdcReport__FIELD_NAME__raw_data, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_ARRAY,
      12,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AdcReport__FIELD_NAME__resolution, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AdcReport__FIELD_NAME__v_ref, 5, 5},
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
px4_msgs__msg__AdcReport__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__AdcReport__TYPE_NAME, 22, 22},
      {px4_msgs__msg__AdcReport__FIELDS, 6, 6},
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
  "uint32 device_id\\t\\t# unique device ID for the sensor that does not change between power cycles\n"
  "int16[12] channel_id\\t\\t# ADC channel IDs, negative for non-existent, TODO: should be kept same as array index\n"
  "int32[12] raw_data\\t\\t# ADC channel raw value, accept negative value, valid if channel ID is positive\n"
  "uint32 resolution\\t\\t# ADC channel resolution\n"
  "float32 v_ref\\t\\t\\t# ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__AdcReport__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__AdcReport__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 506, 506},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__AdcReport__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__AdcReport__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
