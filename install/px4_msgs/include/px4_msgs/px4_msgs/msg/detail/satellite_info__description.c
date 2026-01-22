// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/SatelliteInfo.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/satellite_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__SatelliteInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf1, 0xf2, 0x7c, 0xc2, 0x05, 0x46, 0xf1, 0x93,
      0x60, 0x88, 0xd6, 0x88, 0x40, 0x9a, 0xad, 0x52,
      0x2e, 0x9e, 0x6d, 0x92, 0x73, 0xca, 0x44, 0xbf,
      0x32, 0x1a, 0xf3, 0x37, 0xd2, 0xa1, 0x2d, 0x7e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__SatelliteInfo__TYPE_NAME[] = "px4_msgs/msg/SatelliteInfo";

// Define type names, field names, and default values
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__count[] = "count";
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__svid[] = "svid";
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__used[] = "used";
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__elevation[] = "elevation";
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__azimuth[] = "azimuth";
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__snr[] = "snr";
static char px4_msgs__msg__SatelliteInfo__FIELD_NAME__prn[] = "prn";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__SatelliteInfo__FIELDS[] = {
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__count, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__svid, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__used, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__elevation, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__azimuth, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__snr, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__SatelliteInfo__FIELD_NAME__prn, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__SatelliteInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__SatelliteInfo__TYPE_NAME, 26, 26},
      {px4_msgs__msg__SatelliteInfo__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint8 SAT_INFO_MAX_SATELLITES = 40\n"
  "\n"
  "uint8 count\\t\\t\\t# Number of satellites visible to the receiver\n"
  "uint8[40] svid\\t \\t\\t# Space vehicle ID [1..255], see scheme below\n"
  "uint8[40] used\\t\\t\\t# 0: Satellite not used, 1: used for navigation\n"
  "uint8[40] elevation\\t\\t# Elevation (0: right on top of receiver, 90: on the horizon) of satellite\n"
  "uint8[40] azimuth\\t\\t# Direction of satellite, 0: 0 deg, 255: 360 deg.\n"
  "uint8[40] snr\\t\\t\\t# dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite.\n"
  "uint8[40] prn                   # Satellite PRN code assignment, (psuedorandom number SBAS, valid codes are 120-144)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__SatelliteInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__SatelliteInfo__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 684, 684},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__SatelliteInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__SatelliteInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
