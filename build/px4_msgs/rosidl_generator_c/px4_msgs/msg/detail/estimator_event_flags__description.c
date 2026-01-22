// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorEventFlags.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_event_flags__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorEventFlags__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc1, 0xe2, 0x85, 0xda, 0x58, 0x18, 0x6c, 0x97,
      0x57, 0x13, 0xe4, 0x6e, 0x25, 0xeb, 0xc7, 0x4b,
      0xe3, 0x8c, 0x9a, 0xc6, 0x9f, 0x2b, 0x8a, 0xff,
      0xb8, 0x9d, 0xf4, 0xd1, 0x06, 0x78, 0x29, 0x4e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorEventFlags__TYPE_NAME[] = "px4_msgs/msg/EstimatorEventFlags";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__information_event_changes[] = "information_event_changes";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_checks_passed[] = "gps_checks_passed";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_gps[] = "reset_vel_to_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_flow[] = "reset_vel_to_flow";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_vision[] = "reset_vel_to_vision";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_zero[] = "reset_vel_to_zero";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_last_known[] = "reset_pos_to_last_known";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_gps[] = "reset_pos_to_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_vision[] = "reset_pos_to_vision";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_gps_fusion[] = "starting_gps_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_pos_fusion[] = "starting_vision_pos_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_vel_fusion[] = "starting_vision_vel_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_yaw_fusion[] = "starting_vision_yaw_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__yaw_aligned_to_imu_gps[] = "yaw_aligned_to_imu_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_baro[] = "reset_hgt_to_baro";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_gps[] = "reset_hgt_to_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_rng[] = "reset_hgt_to_rng";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_ev[] = "reset_hgt_to_ev";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorEventFlags__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__information_event_changes, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_checks_passed, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_gps, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_flow, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_vision, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_zero, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_last_known, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_gps, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_vision, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_gps_fusion, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_pos_fusion, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_vel_fusion, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_yaw_fusion, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__yaw_aligned_to_imu_gps, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_baro, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_gps, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_rng, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_ev, 15, 15},
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
px4_msgs__msg__EstimatorEventFlags__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorEventFlags__TYPE_NAME, 32, 32},
      {px4_msgs__msg__EstimatorEventFlags__FIELDS, 20, 20},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                        # time since system start (microseconds)\n"
  "uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)\n"
  "\n"
  "# information events\n"
  "uint32 information_event_changes        # number of information event changes\n"
  "bool gps_checks_passed                  #  0 - true when gps quality checks are passing passed\n"
  "bool reset_vel_to_gps                   #  1 - true when the velocity states are reset to the gps measurement\n"
  "bool reset_vel_to_flow                  #  2 - true when the velocity states are reset using the optical flow measurement\n"
  "bool reset_vel_to_vision                #  3 - true when the velocity states are reset to the vision system measurement\n"
  "bool reset_vel_to_zero                  #  4 - true when the velocity states are reset to zero\n"
  "bool reset_pos_to_last_known            #  5 - true when the position states are reset to the last known position\n"
  "bool reset_pos_to_gps                   #  6 - true when the position states are reset to the gps measurement\n"
  "bool reset_pos_to_vision                #  7 - true when the position states are reset to the vision system measurement\n"
  "bool starting_gps_fusion                #  8 - true when the filter starts using gps measurements to correct the state estimates\n"
  "bool starting_vision_pos_fusion         #  9 - true when the filter starts using vision system position measurements to correct the state estimates\n"
  "bool starting_vision_vel_fusion         # 10 - true when the filter starts using vision system velocity measurements to correct the state estimates\n"
  "bool starting_vision_yaw_fusion         # 11 - true when the filter starts using vision system yaw  measurements to correct the state estimates\n"
  "bool yaw_aligned_to_imu_gps             # 12 - true when the filter resets the yaw to an estimate derived from IMU and GPS data\n"
  "bool reset_hgt_to_baro                  # 13 - true when the vertical position state is reset to the baro measurement\n"
  "bool reset_hgt_to_gps                   # 14 - true when the vertical position state is reset to the gps measurement\n"
  "bool reset_hgt_to_rng                   # 15 - true when the vertical position state is reset to the rng measurement\n"
  "bool reset_hgt_to_ev                    # 16 - true when the vertical position state is reset to the ev measurement";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorEventFlags__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorEventFlags__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2319, 2319},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorEventFlags__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorEventFlags__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
