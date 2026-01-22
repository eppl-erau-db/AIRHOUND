// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ActuatorOutputs.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/actuator_outputs__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ActuatorOutputs__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x20, 0xc8, 0x12, 0x1f, 0x7b, 0xcd, 0xf2, 0x56,
      0xc0, 0x58, 0x84, 0xc2, 0x38, 0x80, 0x85, 0xbc,
      0x3f, 0x1e, 0xe0, 0x30, 0xb9, 0x07, 0x7c, 0x9b,
      0xd9, 0x58, 0xb3, 0x61, 0x02, 0x2b, 0x0d, 0x68,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ActuatorOutputs__TYPE_NAME[] = "px4_msgs/msg/ActuatorOutputs";

// Define type names, field names, and default values
static char px4_msgs__msg__ActuatorOutputs__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ActuatorOutputs__FIELD_NAME__noutputs[] = "noutputs";
static char px4_msgs__msg__ActuatorOutputs__FIELD_NAME__output[] = "output";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ActuatorOutputs__FIELDS[] = {
  {
    {px4_msgs__msg__ActuatorOutputs__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorOutputs__FIELD_NAME__noutputs, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorOutputs__FIELD_NAME__output, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      16,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ActuatorOutputs__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ActuatorOutputs__TYPE_NAME, 28, 28},
      {px4_msgs__msg__ActuatorOutputs__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t\\t\\t# time since system start (microseconds)\n"
  "uint8 NUM_ACTUATOR_OUTPUTS\\t\\t= 16\n"
  "uint8 NUM_ACTUATOR_OUTPUT_GROUPS\\t= 4\\t# for sanity checking\n"
  "uint32 noutputs\\t\\t\\t\\t# valid outputs\n"
  "float32[16] output\\t\\t\\t\\t# output data, in natural output units\n"
  "\n"
  "# actuator_outputs_sim is used for SITL, HITL & SIH (with an output range of [-1, 1])\n"
  "# TOPICS actuator_outputs actuator_outputs_sim actuator_outputs_debug";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ActuatorOutputs__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ActuatorOutputs__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 406, 406},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ActuatorOutputs__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ActuatorOutputs__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
