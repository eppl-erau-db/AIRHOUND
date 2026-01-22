// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleOpticalFlow.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_optical_flow__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleOpticalFlow__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe5, 0x79, 0xc7, 0x3b, 0x4a, 0x86, 0x39, 0x5a,
      0x8c, 0x21, 0x0b, 0xbb, 0x66, 0x38, 0x9b, 0xe0,
      0xa2, 0x25, 0x5f, 0x27, 0xdb, 0x3d, 0xd2, 0xb2,
      0xaf, 0x7b, 0x7a, 0x12, 0x17, 0x2c, 0x97, 0x37,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleOpticalFlow__TYPE_NAME[] = "px4_msgs/msg/VehicleOpticalFlow";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__pixel_flow[] = "pixel_flow";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__delta_angle[] = "delta_angle";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__distance_m[] = "distance_m";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__integration_timespan_us[] = "integration_timespan_us";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__quality[] = "quality";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__max_flow_rate[] = "max_flow_rate";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__min_ground_distance[] = "min_ground_distance";
static char px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__max_ground_distance[] = "max_ground_distance";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleOpticalFlow__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__pixel_flow, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__delta_angle, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__distance_m, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__integration_timespan_us, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__quality, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__max_flow_rate, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__min_ground_distance, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlow__FIELD_NAME__max_ground_distance, 19, 19},
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
px4_msgs__msg__VehicleOpticalFlow__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleOpticalFlow__TYPE_NAME, 31, 31},
      {px4_msgs__msg__VehicleOpticalFlow__FIELDS, 11, 11},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Optical flow in XYZ body frame in SI units.\n"
  "\n"
  "uint64 timestamp               # time since system start (microseconds)\n"
  "uint64 timestamp_sample\n"
  "\n"
  "uint32 device_id               # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "float32[2] pixel_flow          # (radians) accumulated optical flow in radians where a positive value is produced by a RH rotation about the body axis\n"
  "\n"
  "float32[3] delta_angle         # (radians) accumulated gyro radians where a positive value is produced by a RH rotation of the sensor about the body axis. (NAN if unavailable)\n"
  "\n"
  "float32 distance_m             # (meters) Distance to the center of the flow field (NAN if unavailable)\n"
  "\n"
  "uint32 integration_timespan_us # (microseconds) accumulation timespan in microseconds\n"
  "\n"
  "uint8 quality                  # Average of quality of accumulated frames, 0: bad quality, 255: maximum quality\n"
  "\n"
  "float32 max_flow_rate          # (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably\n"
  "\n"
  "float32 min_ground_distance    # (meters) Minimum distance from ground at which the optical flow sensor operates reliably\n"
  "float32 max_ground_distance    # (meters) Maximum distance from ground at which the optical flow sensor operates reliably";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleOpticalFlow__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleOpticalFlow__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1256, 1256},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleOpticalFlow__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleOpticalFlow__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
