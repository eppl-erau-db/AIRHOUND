// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorBias3d.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_bias3d__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorBias3d__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9b, 0x78, 0x6c, 0x29, 0x7d, 0xf1, 0x98, 0xa6,
      0x4f, 0xb4, 0xfd, 0x7e, 0xe3, 0x4e, 0xb4, 0xb7,
      0x85, 0x72, 0xaa, 0xe8, 0x65, 0x8d, 0x13, 0xb5,
      0x87, 0xb3, 0x37, 0x0b, 0x93, 0x51, 0xce, 0x0d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorBias3d__TYPE_NAME[] = "px4_msgs/msg/EstimatorBias3d";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__bias[] = "bias";
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__bias_var[] = "bias_var";
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__innov[] = "innov";
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__innov_var[] = "innov_var";
static char px4_msgs__msg__EstimatorBias3d__FIELD_NAME__innov_test_ratio[] = "innov_test_ratio";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorBias3d__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__bias, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__bias_var, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__innov, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__innov_var, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorBias3d__FIELD_NAME__innov_test_ratio, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__EstimatorBias3d__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorBias3d__TYPE_NAME, 28, 28},
      {px4_msgs__msg__EstimatorBias3d__FIELDS, 8, 8},
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
  "uint32 device_id                # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "float32[3] bias                 # estimated barometric altitude bias (m)\n"
  "float32[3] bias_var             # estimated barometric altitude bias variance (m^2)\n"
  "\n"
  "float32[3] innov                # innovation of the last measurement fusion (m)\n"
  "float32[3] innov_var            # innovation variance of the last measurement fusion (m^2)\n"
  "float32[3] innov_test_ratio     # normalized innovation squared test ratio\n"
  "\n"
  "# TOPICS estimator_bias3d\n"
  "# TOPICS estimator_ev_pos_bias";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorBias3d__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorBias3d__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 724, 724},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorBias3d__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorBias3d__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
