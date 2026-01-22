// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Ekf2Timestamps.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/ekf2_timestamps__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Ekf2Timestamps__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5a, 0xce, 0xea, 0x90, 0x4d, 0xd3, 0xe5, 0x61,
      0x4a, 0x8b, 0x42, 0x91, 0x69, 0x0c, 0x29, 0xe4,
      0x4d, 0xe4, 0xb7, 0xf5, 0xf6, 0xd0, 0x90, 0x3a,
      0x78, 0xdf, 0xa1, 0xe3, 0x4b, 0xef, 0xb9, 0xa7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Ekf2Timestamps__TYPE_NAME[] = "px4_msgs/msg/Ekf2Timestamps";

// Define type names, field names, and default values
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__airspeed_timestamp_rel[] = "airspeed_timestamp_rel";
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__airspeed_validated_timestamp_rel[] = "airspeed_validated_timestamp_rel";
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__distance_sensor_timestamp_rel[] = "distance_sensor_timestamp_rel";
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__optical_flow_timestamp_rel[] = "optical_flow_timestamp_rel";
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__vehicle_air_data_timestamp_rel[] = "vehicle_air_data_timestamp_rel";
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__vehicle_magnetometer_timestamp_rel[] = "vehicle_magnetometer_timestamp_rel";
static char px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__visual_odometry_timestamp_rel[] = "visual_odometry_timestamp_rel";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Ekf2Timestamps__FIELDS[] = {
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__airspeed_timestamp_rel, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__airspeed_validated_timestamp_rel, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__distance_sensor_timestamp_rel, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__optical_flow_timestamp_rel, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__vehicle_air_data_timestamp_rel, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__vehicle_magnetometer_timestamp_rel, 34, 34},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Ekf2Timestamps__FIELD_NAME__visual_odometry_timestamp_rel, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__Ekf2Timestamps__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Ekf2Timestamps__TYPE_NAME, 27, 27},
      {px4_msgs__msg__Ekf2Timestamps__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# this message contains the (relative) timestamps of the sensor inputs used by EKF2.\n"
  "# It can be used for reproducible replay.\n"
  "\n"
  "# the timestamp field is the ekf2 reference time and matches the timestamp of\n"
  "# the sensor_combined topic.\n"
  "\n"
  "uint64 timestamp\\t\\t\\t # time since system start (microseconds)\n"
  "\n"
  "int16 RELATIVE_TIMESTAMP_INVALID = 32767 # (0x7fff) If one of the relative timestamps\n"
  "                                         # is set to this value, it means the associated sensor values did not update\n"
  "\n"
  "# timestamps are relative to the main timestamp and are in 0.1 ms (timestamp +\n"
  "# *_timestamp_rel = absolute timestamp). For int16, this allows a maximum\n"
  "# difference of +-3.2s to the sensor_combined topic.\n"
  "\n"
  "int16 airspeed_timestamp_rel\n"
  "int16 airspeed_validated_timestamp_rel\n"
  "int16 distance_sensor_timestamp_rel\n"
  "int16 optical_flow_timestamp_rel\n"
  "int16 vehicle_air_data_timestamp_rel\n"
  "int16 vehicle_magnetometer_timestamp_rel\n"
  "int16 visual_odometry_timestamp_rel\n"
  "\n"
  "# Note: this is a high-rate logged topic, so it needs to be as small as possible";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Ekf2Timestamps__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Ekf2Timestamps__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1043, 1043},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Ekf2Timestamps__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Ekf2Timestamps__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
