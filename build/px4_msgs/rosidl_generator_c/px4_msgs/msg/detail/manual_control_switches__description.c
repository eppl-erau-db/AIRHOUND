// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ManualControlSwitches.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/manual_control_switches__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ManualControlSwitches__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x68, 0x51, 0xdb, 0x1a, 0x96, 0x9e, 0x76, 0xf6,
      0x49, 0xcb, 0xb9, 0x1d, 0x3c, 0x1f, 0x17, 0xa2,
      0x92, 0x93, 0x66, 0xd3, 0x94, 0xc6, 0x84, 0x43,
      0xfc, 0xc8, 0x3d, 0x8f, 0xd1, 0xc2, 0x47, 0x34,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ManualControlSwitches__TYPE_NAME[] = "px4_msgs/msg/ManualControlSwitches";

// Define type names, field names, and default values
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__mode_slot[] = "mode_slot";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__arm_switch[] = "arm_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__return_switch[] = "return_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__loiter_switch[] = "loiter_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__offboard_switch[] = "offboard_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__kill_switch[] = "kill_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__termination_switch[] = "termination_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__gear_switch[] = "gear_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__transition_switch[] = "transition_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__photo_switch[] = "photo_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__video_switch[] = "video_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__engage_main_motor_switch[] = "engage_main_motor_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__payload_power_switch[] = "payload_power_switch";
static char px4_msgs__msg__ManualControlSwitches__FIELD_NAME__switch_changes[] = "switch_changes";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ManualControlSwitches__FIELDS[] = {
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__mode_slot, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__arm_switch, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__return_switch, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__loiter_switch, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__offboard_switch, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__kill_switch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__termination_switch, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__gear_switch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__transition_switch, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__photo_switch, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__video_switch, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__engage_main_motor_switch, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__payload_power_switch, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ManualControlSwitches__FIELD_NAME__switch_changes, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ManualControlSwitches__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ManualControlSwitches__TYPE_NAME, 34, 34},
      {px4_msgs__msg__ManualControlSwitches__FIELDS, 16, 16},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                 # time since system start (microseconds)\n"
  "\n"
  "uint64 timestamp_sample          # the timestamp of the raw data (microseconds)\n"
  "\n"
  "uint8 SWITCH_POS_NONE   = 0      # switch is not mapped\n"
  "uint8 SWITCH_POS_ON     = 1      # switch activated (value = 1)\n"
  "uint8 SWITCH_POS_MIDDLE = 2      # middle position (value = 0)\n"
  "uint8 SWITCH_POS_OFF    = 3      # switch not activated (value = -1)\n"
  "\n"
  "uint8 MODE_SLOT_NONE    = 0      # no mode slot assigned\n"
  "uint8 MODE_SLOT_1       = 1      # mode slot 1 selected\n"
  "uint8 MODE_SLOT_2       = 2      # mode slot 2 selected\n"
  "uint8 MODE_SLOT_3       = 3      # mode slot 3 selected\n"
  "uint8 MODE_SLOT_4       = 4      # mode slot 4 selected\n"
  "uint8 MODE_SLOT_5       = 5      # mode slot 5 selected\n"
  "uint8 MODE_SLOT_6       = 6      # mode slot 6 selected\n"
  "uint8 MODE_SLOT_NUM     = 6      # number of slots\n"
  "\n"
  "uint8 mode_slot                  # the slot a specific model selector is in\n"
  "\n"
  "uint8 arm_switch                 # arm/disarm switch: _DISARMED_, ARMED\n"
  "uint8 return_switch              # return to launch 2 position switch (mandatory): _NORMAL_, RTL\n"
  "uint8 loiter_switch              # loiter 2 position switch (optional): _MISSION_, LOITER\n"
  "uint8 offboard_switch            # offboard 2 position switch (optional): _NORMAL_, OFFBOARD\n"
  "uint8 kill_switch                # throttle kill: _NORMAL_, KILL\n"
  "uint8 termination_switch         # trigger termination which cannot be undone\n"
  "uint8 gear_switch                # landing gear switch: _DOWN_, UP\n"
  "uint8 transition_switch          # VTOL transition switch: _HOVER, FORWARD_FLIGHT\n"
  "\n"
  "uint8 photo_switch               # Photo trigger switch\n"
  "uint8 video_switch               # Photo trigger switch\n"
  "\n"
  "uint8 engage_main_motor_switch   # Engage the main motor (for helicopters)\n"
  "\n"
  "uint8 payload_power_switch       # Payload power switch\n"
  "\n"
  "uint32 switch_changes            # number of switch changes";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ManualControlSwitches__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ManualControlSwitches__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1882, 1882},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ManualControlSwitches__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ManualControlSwitches__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
