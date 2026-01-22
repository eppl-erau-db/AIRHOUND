// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SensorGnssStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/sensor_gnss_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SensorGnssStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x17, 0xeb, 0xac, 0x3d, 0x2e, 0x46, 0x73, 0x5d,
      0x70, 0x10, 0x49, 0x05, 0xbe, 0xea, 0x11, 0xcb,
      0x80, 0x75, 0xb1, 0x1d, 0x8c, 0x39, 0xa2, 0x50,
      0x69, 0xb9, 0x83, 0xe6, 0xca, 0x3a, 0x01, 0x2a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SensorGnssStatus__TYPE_NAME[] = "px4_msgs/msg/SensorGnssStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__SensorGnssStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SensorGnssStatus__FIELD_NAME__device_id[] = "device_id";
static char px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_available[] = "quality_available";
static char px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_corrections[] = "quality_corrections";
static char px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_receiver[] = "quality_receiver";
static char px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_gnss_signals[] = "quality_gnss_signals";
static char px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_post_processing[] = "quality_post_processing";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SensorGnssStatus__FIELDS[] = {
  {
    {px4_msgs__msg__SensorGnssStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssStatus__FIELD_NAME__device_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_available, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_corrections, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_receiver, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_gnss_signals, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SensorGnssStatus__FIELD_NAME__quality_post_processing, 23, 23},
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
px4_msgs__msg__SensorGnssStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SensorGnssStatus__TYPE_NAME, 29, 29},
      {px4_msgs__msg__SensorGnssStatus__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Gnss quality indicators\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint32 device_id                # unique device ID for the sensor that does not change between power cycles\n"
  "\n"
  "bool  quality_available         # Set to true if quality indicators are available\n"
  "uint8 quality_corrections       # Corrections quality from 0 to 10, or 255 if not available\n"
  "uint8 quality_receiver          # Overall receiver operating status from 0 to 10, or 255 if not available\n"
  "uint8 quality_gnss_signals      # Quality of GNSS signals from 0 to 10, or 255 if not available\n"
  "uint8 quality_post_processing   # Expected post processing quality from 0 to 10, or 255 if not available";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SensorGnssStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SensorGnssStatus__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 676, 676},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SensorGnssStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SensorGnssStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
