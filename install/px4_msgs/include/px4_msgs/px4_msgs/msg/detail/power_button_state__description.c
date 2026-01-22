// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/PowerButtonState.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/power_button_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__PowerButtonState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9f, 0xcd, 0x29, 0x4f, 0xa3, 0xa9, 0x8e, 0x7c,
      0xd7, 0x24, 0x6e, 0x7e, 0x92, 0x1b, 0x8c, 0xf3,
      0x20, 0xb7, 0x88, 0xd7, 0xcf, 0x02, 0x84, 0x4f,
      0x88, 0x7f, 0xbd, 0x99, 0xeb, 0x3b, 0xc6, 0xba,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__PowerButtonState__TYPE_NAME[] = "px4_msgs/msg/PowerButtonState";

// Define type names, field names, and default values
static char px4_msgs__msg__PowerButtonState__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__PowerButtonState__FIELD_NAME__event[] = "event";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__PowerButtonState__FIELDS[] = {
  {
    {px4_msgs__msg__PowerButtonState__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PowerButtonState__FIELD_NAME__event, 5, 5},
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
px4_msgs__msg__PowerButtonState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__PowerButtonState__TYPE_NAME, 29, 29},
      {px4_msgs__msg__PowerButtonState__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# power button state notification message\n"
  "\n"
  "uint64 timestamp\\t\\t\\t    # time since system start (microseconds)\n"
  "\n"
  "uint8 PWR_BUTTON_STATE_IDEL = 0             # Button went up without meeting shutdown button down time (delete event)\n"
  "uint8 PWR_BUTTON_STATE_DOWN = 1             # Button went Down\n"
  "uint8 PWR_BUTTON_STATE_UP = 2               # Button went Up\n"
  "uint8 PWR_BUTTON_STATE_REQUEST_SHUTDOWN = 3 # Button went Up after meeting shutdown button down time\n"
  "\n"
  "uint8 event                                 # one of PWR_BUTTON_STATE_*";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__PowerButtonState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__PowerButtonState__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 524, 524},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__PowerButtonState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__PowerButtonState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
