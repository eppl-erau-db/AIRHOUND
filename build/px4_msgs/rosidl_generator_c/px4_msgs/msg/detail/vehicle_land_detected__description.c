// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleLandDetected.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_land_detected__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleLandDetected__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xce, 0xd2, 0x2f, 0xc1, 0x69, 0x9e, 0x2a, 0x37,
      0x46, 0xa1, 0xfa, 0x86, 0xaa, 0x4b, 0x54, 0x63,
      0xb4, 0x2f, 0x5b, 0x9c, 0x97, 0xe7, 0x91, 0x30,
      0x63, 0x36, 0x5a, 0x04, 0xd8, 0x32, 0x49, 0x01,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleLandDetected__TYPE_NAME[] = "px4_msgs/msg/VehicleLandDetected";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__freefall[] = "freefall";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__ground_contact[] = "ground_contact";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__maybe_landed[] = "maybe_landed";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__landed[] = "landed";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__in_ground_effect[] = "in_ground_effect";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__in_descend[] = "in_descend";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__has_low_throttle[] = "has_low_throttle";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__vertical_movement[] = "vertical_movement";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__horizontal_movement[] = "horizontal_movement";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__rotational_movement[] = "rotational_movement";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__close_to_ground_or_skipped_check[] = "close_to_ground_or_skipped_check";
static char px4_msgs__msg__VehicleLandDetected__FIELD_NAME__at_rest[] = "at_rest";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleLandDetected__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__freefall, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__ground_contact, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__maybe_landed, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__landed, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__in_ground_effect, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__in_descend, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__has_low_throttle, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__vertical_movement, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__horizontal_movement, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__rotational_movement, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__close_to_ground_or_skipped_check, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLandDetected__FIELD_NAME__at_rest, 7, 7},
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
px4_msgs__msg__VehicleLandDetected__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleLandDetected__TYPE_NAME, 32, 32},
      {px4_msgs__msg__VehicleLandDetected__FIELDS, 13, 13},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "\n"
  "bool freefall\\t\\t# true if vehicle is currently in free-fall\n"
  "bool ground_contact\\t# true if vehicle has ground contact but is not landed (1. stage)\n"
  "bool maybe_landed\\t# true if the vehicle might have landed (2. stage)\n"
  "bool landed\\t\\t# true if vehicle is currently landed on the ground (3. stage)\n"
  "\n"
  "bool in_ground_effect # indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing).\n"
  "bool in_descend\n"
  "\n"
  "bool has_low_throttle\n"
  "\n"
  "bool vertical_movement\n"
  "bool horizontal_movement\n"
  "bool rotational_movement\n"
  "\n"
  "bool close_to_ground_or_skipped_check\n"
  "\n"
  "bool at_rest";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleLandDetected__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleLandDetected__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 800, 800},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleLandDetected__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleLandDetected__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
