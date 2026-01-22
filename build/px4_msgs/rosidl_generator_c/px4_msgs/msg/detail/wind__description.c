// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Wind.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/wind__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Wind__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf2, 0xf1, 0x3d, 0x3b, 0x63, 0xa7, 0x5b, 0xc0,
      0xce, 0x93, 0x93, 0x89, 0x1d, 0x5d, 0x64, 0xcb,
      0xbc, 0x21, 0x12, 0x69, 0x00, 0xd0, 0x0f, 0x68,
      0x40, 0x60, 0x9e, 0x6b, 0xd5, 0x92, 0x96, 0x2f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Wind__TYPE_NAME[] = "px4_msgs/msg/Wind";

// Define type names, field names, and default values
static char px4_msgs__msg__Wind__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Wind__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__Wind__FIELD_NAME__windspeed_north[] = "windspeed_north";
static char px4_msgs__msg__Wind__FIELD_NAME__windspeed_east[] = "windspeed_east";
static char px4_msgs__msg__Wind__FIELD_NAME__variance_north[] = "variance_north";
static char px4_msgs__msg__Wind__FIELD_NAME__variance_east[] = "variance_east";
static char px4_msgs__msg__Wind__FIELD_NAME__tas_innov[] = "tas_innov";
static char px4_msgs__msg__Wind__FIELD_NAME__tas_innov_var[] = "tas_innov_var";
static char px4_msgs__msg__Wind__FIELD_NAME__beta_innov[] = "beta_innov";
static char px4_msgs__msg__Wind__FIELD_NAME__beta_innov_var[] = "beta_innov_var";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Wind__FIELDS[] = {
  {
    {px4_msgs__msg__Wind__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__windspeed_north, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__windspeed_east, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__variance_north, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__variance_east, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__tas_innov, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__tas_innov_var, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__beta_innov, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Wind__FIELD_NAME__beta_innov_var, 14, 14},
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
px4_msgs__msg__Wind__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Wind__TYPE_NAME, 17, 17},
      {px4_msgs__msg__Wind__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Wind estimate (from EKF2)\n"
  "#\n"
  "# Contains the system-wide estimate of horizontal wind velocity and its variance.\n"
  "# Published by the navigation filter (EKF2) for use by other flight modules and libraries.\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp # [us] Time since system start\n"
  "uint64 timestamp_sample # [us] Timestamp of the raw data\n"
  "\n"
  "float32 windspeed_north # [m/s] Wind component in north / X direction\n"
  "float32 windspeed_east # [m/s] Wind component in east / Y direction\n"
  "\n"
  "float32 variance_north # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in north / X direction\n"
  "float32 variance_east # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in east / Y direction\n"
  "\n"
  "float32 tas_innov # [m/s] True airspeed innovation\n"
  "float32 tas_innov_var # [(m/s)^2] True airspeed innovation variance\n"
  "\n"
  "float32 beta_innov # [rad] Sideslip measurement innovation\n"
  "float32 beta_innov_var # [rad^2] Sideslip measurement innovation variance\n"
  "\n"
  "# TOPICS wind estimator_wind";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Wind__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Wind__TYPE_NAME, 17, 17},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 993, 993},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Wind__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Wind__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
