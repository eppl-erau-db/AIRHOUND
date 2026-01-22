// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleAttitude.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_attitude__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleAttitude__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0xf7, 0x4a, 0xc6, 0xc0, 0x13, 0x35, 0xf3,
      0xa8, 0x61, 0x2e, 0x35, 0x79, 0xd4, 0xea, 0xa8,
      0x9a, 0x9d, 0x81, 0x55, 0x0d, 0x93, 0x3e, 0xd0,
      0x57, 0xf5, 0xd6, 0x25, 0x2d, 0xc7, 0xe5, 0x0c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleAttitude__TYPE_NAME[] = "px4_msgs/msg/VehicleAttitude";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleAttitude__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleAttitude__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleAttitude__FIELD_NAME__q[] = "q";
static char px4_msgs__msg__VehicleAttitude__FIELD_NAME__delta_q_reset[] = "delta_q_reset";
static char px4_msgs__msg__VehicleAttitude__FIELD_NAME__quat_reset_counter[] = "quat_reset_counter";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleAttitude__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleAttitude__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAttitude__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAttitude__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAttitude__FIELD_NAME__delta_q_reset, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAttitude__FIELD_NAME__quat_reset_counter, 18, 18},
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
px4_msgs__msg__VehicleAttitude__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleAttitude__TYPE_NAME, 28, 28},
      {px4_msgs__msg__VehicleAttitude__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use\n"
  "# The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)\n"
  "\n"
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp                # time since system start (microseconds)\n"
  "\n"
  "uint64 timestamp_sample         # the timestamp of the raw data (microseconds)\n"
  "\n"
  "float32[4] q                    # Quaternion rotation from the FRD body frame to the NED earth frame\n"
  "float32[4] delta_q_reset        # Amount by which quaternion has changed during last reset\n"
  "uint8 quat_reset_counter        # Quaternion reset counter\n"
  "\n"
  "# TOPICS vehicle_attitude vehicle_attitude_groundtruth external_ins_attitude\n"
  "# TOPICS estimator_attitude";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleAttitude__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleAttitude__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 700, 700},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleAttitude__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleAttitude__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
