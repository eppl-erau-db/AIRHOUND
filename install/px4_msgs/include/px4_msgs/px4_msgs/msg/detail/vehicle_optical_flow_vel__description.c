// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleOpticalFlowVel.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_optical_flow_vel__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleOpticalFlowVel__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe9, 0x0f, 0x7f, 0x0b, 0x44, 0x74, 0x7c, 0xf7,
      0x14, 0x0e, 0xbb, 0xf7, 0xc7, 0x6c, 0xdf, 0x33,
      0x4b, 0x14, 0x3b, 0xaa, 0x28, 0x7a, 0x02, 0x98,
      0x3f, 0x0e, 0x44, 0xb0, 0x19, 0xc1, 0x3b, 0x56,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleOpticalFlowVel__TYPE_NAME[] = "px4_msgs/msg/VehicleOpticalFlowVel";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_body[] = "vel_body";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_ne[] = "vel_ne";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_body_filtered[] = "vel_body_filtered";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_ne_filtered[] = "vel_ne_filtered";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__flow_rate_uncompensated[] = "flow_rate_uncompensated";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__flow_rate_compensated[] = "flow_rate_compensated";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__gyro_rate[] = "gyro_rate";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__gyro_bias[] = "gyro_bias";
static char px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__ref_gyro[] = "ref_gyro";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleOpticalFlowVel__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_body, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_ne, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_body_filtered, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__vel_ne_filtered, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__flow_rate_uncompensated, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__flow_rate_compensated, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__gyro_rate, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__gyro_bias, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleOpticalFlowVel__FIELD_NAME__ref_gyro, 8, 8},
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
px4_msgs__msg__VehicleOpticalFlowVel__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleOpticalFlowVel__TYPE_NAME, 34, 34},
      {px4_msgs__msg__VehicleOpticalFlowVel__FIELDS, 11, 11},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                       # time since system start (microseconds)\n"
  "uint64 timestamp_sample                # the timestamp of the raw data (microseconds)\n"
  "\n"
  "float32[2] vel_body                    # velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)\n"
  "float32[2] vel_ne                      # same as vel_body but in local frame (m/s)\n"
  "\n"
  "float32[2] vel_body_filtered           # filtered velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)\n"
  "float32[2] vel_ne_filtered             # filtered same as vel_body_filtered but in local frame (m/s)\n"
  "\n"
  "float32[2] flow_rate_uncompensated     # integrated optical flow measurement (rad/s)\n"
  "float32[2] flow_rate_compensated       # integrated optical flow measurement compensated for angular motion (rad/s)\n"
  "\n"
  "float32[3] gyro_rate                   # gyro measurement synchronized with flow measurements (rad/s)\n"
  "\n"
  "float32[3] gyro_bias\n"
  "float32[3] ref_gyro\n"
  "\n"
  "# TOPICS estimator_optical_flow_vel vehicle_optical_flow_vel";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleOpticalFlowVel__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleOpticalFlowVel__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1070, 1070},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleOpticalFlowVel__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleOpticalFlowVel__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
