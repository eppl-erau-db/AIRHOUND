// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/airspeed__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__Airspeed__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x27, 0xf7, 0x74, 0x06, 0x2e, 0xbd, 0x92, 0xe8,
      0x4e, 0x22, 0x17, 0x3f, 0xdf, 0x17, 0x29, 0x3e,
      0x77, 0x6c, 0x85, 0x66, 0xac, 0x89, 0x2f, 0xa1,
      0x27, 0xfc, 0x78, 0xdc, 0x8b, 0x57, 0x1c, 0x55,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__Airspeed__TYPE_NAME[] = "px4_msgs/msg/Airspeed";

// Define type names, field names, and default values
static char px4_msgs__msg__Airspeed__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__Airspeed__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__Airspeed__FIELD_NAME__indicated_airspeed_m_s[] = "indicated_airspeed_m_s";
static char px4_msgs__msg__Airspeed__FIELD_NAME__true_airspeed_m_s[] = "true_airspeed_m_s";
static char px4_msgs__msg__Airspeed__FIELD_NAME__confidence[] = "confidence";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__Airspeed__FIELDS[] = {
  {
    {px4_msgs__msg__Airspeed__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Airspeed__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Airspeed__FIELD_NAME__indicated_airspeed_m_s, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Airspeed__FIELD_NAME__true_airspeed_m_s, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__Airspeed__FIELD_NAME__confidence, 10, 10},
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
px4_msgs__msg__Airspeed__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__Airspeed__TYPE_NAME, 21, 21},
      {px4_msgs__msg__Airspeed__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Airspeed data from sensors\n"
  "#\n"
  "# This is published by airspeed sensor drivers, CAN airspeed sensors, simulators.\n"
  "# It is subscribed by the airspeed selector module, which validates the data from multiple sensors and passes on a single estimation to the EKF, controllers and telemetry providers.\n"
  "\n"
  "uint64 timestamp                 # [us] Time since system start\n"
  "uint64 timestamp_sample          # [us] Timestamp of the raw data\n"
  "float32 indicated_airspeed_m_s   # [m/s] Indicated airspeed\n"
  "float32 true_airspeed_m_s        # [m/s] True airspeed\n"
  "float32 confidence               # [@range 0,1] Confidence value for this sensor";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__Airspeed__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__Airspeed__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 622, 622},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__Airspeed__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__Airspeed__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
