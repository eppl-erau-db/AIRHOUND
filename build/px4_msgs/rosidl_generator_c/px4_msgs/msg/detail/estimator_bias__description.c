// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorBias.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_bias__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorBias__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x19, 0x89, 0xf7, 0x7c, 0xb0, 0x4d, 0x49, 0x64,
      0x89, 0xae, 0xca, 0xe0, 0x9b, 0x7a, 0xef, 0x03,
      0x1c, 0x02, 0xad, 0x6b, 0x54, 0x16, 0x48, 0xf0,
      0x9c, 0xcf, 0x6d, 0xbe, 0xa4, 0x59, 0x7e, 0x8f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorBias__TYPE_NAME[] = "px4_msgs/msg/EstimatorBias";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__bias[] = "bias";
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__bias_var[] = "bias_var";
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__innov[] = "innov";
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__innov_var[] = "innov_var";
static char px4_msgs__msg__EstimatorBias__FIELD_NAME__innov_test_ratio[] = "innov_test_ratio";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorBias__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__bias, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__bias_var, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__innov, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__innov_var, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias__FIELD_NAME__innov_test_ratio, 16, 16},
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
px4_msgs__msg__EstimatorBias__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorBias__TYPE_NAME, 26, 26},
      {px4_msgs__msg__EstimatorBias__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                # time since system start (microseconds)\n"
  "uint64 timestamp_sample         # the timestamp of the raw data (microseconds)\n"
  "\n"
  "uint32 device_id\\t\\t# unique device ID for the sensor that does not change between power cycles\n"
  "float32 bias\\t\\t\\t# estimated barometric altitude bias (m)\n"
  "float32 bias_var\\t\\t# estimated barometric altitude bias variance (m^2)\n"
  "\n"
  "float32 innov\\t\\t\\t# innovation of the last measurement fusion (m)\n"
  "float32 innov_var\\t\\t# innovation variance of the last measurement fusion (m^2)\n"
  "float32 innov_test_ratio\\t# normalized innovation squared test ratio\n"
  "\n"
  "# TOPICS estimator_baro_bias estimator_gnss_hgt_bias";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorBias__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorBias__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 638, 638},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorBias__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorBias__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
