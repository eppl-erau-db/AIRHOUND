// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/AirspeedWind.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/airspeed_wind__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__AirspeedWind__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7d, 0xdf, 0x19, 0x1c, 0x7c, 0xdb, 0x7a, 0x76,
      0x8b, 0xbd, 0x73, 0x3c, 0x14, 0x7d, 0xa1, 0x10,
      0x3e, 0x36, 0x1f, 0xdb, 0xd7, 0x0c, 0xb2, 0xcc,
      0x7c, 0xae, 0x3c, 0x1a, 0x3b, 0xc2, 0x59, 0x46,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__AirspeedWind__TYPE_NAME[] = "px4_msgs/msg/AirspeedWind";

// Define type names, field names, and default values
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__windspeed_north[] = "windspeed_north";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__windspeed_east[] = "windspeed_east";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__variance_north[] = "variance_north";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__variance_east[] = "variance_east";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_innov[] = "tas_innov";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_innov_var[] = "tas_innov_var";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_scale_raw[] = "tas_scale_raw";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_scale_raw_var[] = "tas_scale_raw_var";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_scale_validated[] = "tas_scale_validated";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__beta_innov[] = "beta_innov";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__beta_innov_var[] = "beta_innov_var";
static char px4_msgs__msg__AirspeedWind__FIELD_NAME__source[] = "source";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__AirspeedWind__FIELDS[] = {
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__windspeed_north, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__windspeed_east, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__variance_north, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__variance_east, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_innov, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_innov_var, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_scale_raw, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_scale_raw_var, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__tas_scale_validated, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__beta_innov, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__beta_innov_var, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AirspeedWind__FIELD_NAME__source, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__AirspeedWind__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__AirspeedWind__TYPE_NAME, 25, 25},
      {px4_msgs__msg__AirspeedWind__FIELDS, 14, 14},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Wind estimate (from airspeed_selector)\n"
  "#\n"
  "# Contains wind estimation and airspeed innovation information estimated by the WindEstimator\n"
  "# in the airspeed selector module.\n"
  "#\n"
  "# This message is published by the airspeed selector for debugging purposes, and is not\n"
  "# subscribed to by any other modules.\n"
  "\n"
  "uint64 timestamp # [us] Time since system start\n"
  "uint64 timestamp_sample # [us] Timestamp of the raw data\n"
  "\n"
  "float32 windspeed_north\\t# [m/s] Wind component in north / X direction\n"
  "float32 windspeed_east # [m/s] Wind component in east / Y direction\n"
  "\n"
  "float32 variance_north # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in north / X direction\n"
  "float32 variance_east # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in east / Y direction\n"
  "\n"
  "float32 tas_innov # [m/s] True airspeed innovation\n"
  "float32 tas_innov_var # [m/s] True airspeed innovation variance\n"
  "\n"
  "float32 tas_scale_raw # Estimated true airspeed scale factor (not validated)\n"
  "float32 tas_scale_raw_var # True airspeed scale factor variance\n"
  "\n"
  "float32 tas_scale_validated # Estimated true airspeed scale factor after validation\n"
  "\n"
  "float32 beta_innov # [rad] Sideslip measurement innovation\n"
  "float32 beta_innov_var # [rad^2] Sideslip measurement innovation variance\n"
  "\n"
  "uint8 source # source of wind estimate\n"
  "\n"
  "uint8 SOURCE_AS_BETA_ONLY = 0 # Wind estimate only based on synthetic sideslip fusion\n"
  "uint8 SOURCE_AS_SENSOR_1 = 1 # Combined synthetic sideslip and airspeed fusion (data from first airspeed sensor)\n"
  "uint8 SOURCE_AS_SENSOR_2 = 2 # Combined synthetic sideslip and airspeed fusion (data from second airspeed sensor)\n"
  "uint8 SOURCE_AS_SENSOR_3 = 3 # Combined synthetic sideslip and airspeed fusion (data from third airspeed sensor)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__AirspeedWind__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__AirspeedWind__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1722, 1722},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__AirspeedWind__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__AirspeedWind__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
