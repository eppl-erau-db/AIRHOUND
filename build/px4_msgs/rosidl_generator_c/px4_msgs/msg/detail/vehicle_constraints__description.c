// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleConstraints.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_constraints__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleConstraints__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb0, 0xd6, 0xe3, 0x5e, 0x37, 0xb2, 0xc5, 0xf5,
      0xa4, 0x3c, 0x24, 0xef, 0xd4, 0x43, 0xde, 0x8f,
      0x2f, 0x3c, 0xcb, 0xb2, 0xda, 0x97, 0xb0, 0x73,
      0x6d, 0xbd, 0x25, 0x29, 0x35, 0x1d, 0x05, 0x41,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleConstraints__TYPE_NAME[] = "px4_msgs/msg/VehicleConstraints";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleConstraints__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleConstraints__FIELD_NAME__speed_up[] = "speed_up";
static char px4_msgs__msg__VehicleConstraints__FIELD_NAME__speed_down[] = "speed_down";
static char px4_msgs__msg__VehicleConstraints__FIELD_NAME__want_takeoff[] = "want_takeoff";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleConstraints__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleConstraints__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleConstraints__FIELD_NAME__speed_up, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleConstraints__FIELD_NAME__speed_down, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleConstraints__FIELD_NAME__want_takeoff, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__VehicleConstraints__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleConstraints__TYPE_NAME, 31, 31},
      {px4_msgs__msg__VehicleConstraints__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Local setpoint constraints in NED frame\n"
  "# setting something to NaN means that no limit is provided\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32 speed_up # in meters/sec\n"
  "float32 speed_down # in meters/sec\n"
  "\n"
  "bool want_takeoff # tell the controller to initiate takeoff when idling (ignored during flight)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleConstraints__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleConstraints__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 326, 326},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleConstraints__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleConstraints__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
