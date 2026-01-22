// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/MagWorkerData.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/mag_worker_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__MagWorkerData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0xe3, 0xf3, 0x77, 0x64, 0xfc, 0x56, 0x0b,
      0x15, 0xc0, 0xbf, 0x8e, 0xe5, 0x29, 0x41, 0xbd,
      0x39, 0x3f, 0xeb, 0x66, 0xeb, 0x40, 0x09, 0x63,
      0x2e, 0x44, 0x6b, 0xad, 0x00, 0x82, 0x51, 0x98,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__MagWorkerData__TYPE_NAME[] = "px4_msgs/msg/MagWorkerData";

// Define type names, field names, and default values
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__done_count[] = "done_count";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__calibration_points_perside[] = "calibration_points_perside";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__calibration_interval_perside_us[] = "calibration_interval_perside_us";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__calibration_counter_total[] = "calibration_counter_total";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__side_data_collected[] = "side_data_collected";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__x[] = "x";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__y[] = "y";
static char px4_msgs__msg__MagWorkerData__FIELD_NAME__z[] = "z";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__MagWorkerData__FIELDS[] = {
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__done_count, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__calibration_points_perside, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__calibration_interval_perside_us, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__calibration_counter_total, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__side_data_collected, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__MagWorkerData__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__MagWorkerData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__MagWorkerData__TYPE_NAME, 26, 26},
      {px4_msgs__msg__MagWorkerData__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp          # time since system start (microseconds)\n"
  "uint64 timestamp_sample\n"
  "\n"
  "uint8 MAX_MAGS = 4\n"
  "\n"
  "uint32 done_count\n"
  "uint32 calibration_points_perside\n"
  "uint64 calibration_interval_perside_us\n"
  "uint32[4] calibration_counter_total\n"
  "bool[4] side_data_collected\n"
  "float32[4] x\n"
  "float32[4] y\n"
  "float32[4] z";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__MagWorkerData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__MagWorkerData__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 306, 306},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__MagWorkerData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__MagWorkerData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
