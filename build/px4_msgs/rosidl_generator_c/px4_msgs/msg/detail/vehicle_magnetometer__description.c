// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleMagnetometer.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_magnetometer__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleMagnetometer__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa2, 0x20, 0xeb, 0x9f, 0x50, 0xcd, 0x77, 0x49,
      0x69, 0x57, 0xae, 0x25, 0x57, 0xdc, 0x75, 0xcc,
      0x12, 0x9b, 0x0d, 0xeb, 0x5d, 0xcc, 0x33, 0x9e,
      0x4e, 0x74, 0x66, 0x84, 0xa3, 0x9e, 0x07, 0xa9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleMagnetometer__TYPE_NAME[] = "px4_msgs/msg/VehicleMagnetometer";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__magnetometer_ga[] = "magnetometer_ga";
static char px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__calibration_count[] = "calibration_count";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleMagnetometer__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__magnetometer_ga, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleMagnetometer__FIELD_NAME__calibration_count, 17, 17},
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
px4_msgs__msg__VehicleMagnetometer__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleMagnetometer__TYPE_NAME, 32, 32},
      {px4_msgs__msg__VehicleMagnetometer__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "\n"
  "uint64 timestamp            # time since system start (microseconds)\n"
  "\n"
  "uint64 timestamp_sample     # the timestamp of the raw data (microseconds)\n"
  "\n"
  "uint32 device_id            # unique device ID for the selected magnetometer\n"
  "\n"
  "float32[3] magnetometer_ga  # Magnetic field in the FRD body frame XYZ-axis in Gauss\n"
  "\n"
  "uint8 calibration_count     # Calibration changed counter. Monotonically increases whenever calibration changes.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleMagnetometer__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleMagnetometer__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 424, 424},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleMagnetometer__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleMagnetometer__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
