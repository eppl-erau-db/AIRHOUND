// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/FlightPhaseEstimation.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/flight_phase_estimation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__FlightPhaseEstimation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdf, 0x04, 0x20, 0xcb, 0xb6, 0x87, 0x18, 0xed,
      0xa1, 0x27, 0x3f, 0x37, 0xb2, 0xd3, 0x83, 0xa0,
      0x1d, 0x9b, 0x7c, 0xd1, 0x42, 0xc7, 0xcb, 0x9e,
      0xb2, 0x17, 0x36, 0x4f, 0x53, 0xd2, 0xa3, 0x24,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__FlightPhaseEstimation__TYPE_NAME[] = "px4_msgs/msg/FlightPhaseEstimation";

// Define type names, field names, and default values
static char px4_msgs__msg__FlightPhaseEstimation__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__FlightPhaseEstimation__FIELD_NAME__flight_phase[] = "flight_phase";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__FlightPhaseEstimation__FIELDS[] = {
  {
    {px4_msgs__msg__FlightPhaseEstimation__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__FlightPhaseEstimation__FIELD_NAME__flight_phase, 12, 12},
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
px4_msgs__msg__FlightPhaseEstimation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__FlightPhaseEstimation__TYPE_NAME, 34, 34},
      {px4_msgs__msg__FlightPhaseEstimation__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp               # time since system start (microseconds)\n"
  "\n"
  "uint8 flight_phase \\t\\t# Estimate of current flight phase\n"
  "\n"
  "uint8 FLIGHT_PHASE_UNKNOWN = 0  # vehicle flight phase is unknown\n"
  "uint8 FLIGHT_PHASE_LEVEL = 1\\t# Vehicle is in level flight\n"
  "uint8 FLIGHT_PHASE_DESCEND = 2\\t# vehicle is in descend\n"
  "uint8 FLIGHT_PHASE_CLIMB = 3   # vehicle is climbing";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__FlightPhaseEstimation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__FlightPhaseEstimation__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 362, 362},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__FlightPhaseEstimation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__FlightPhaseEstimation__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
