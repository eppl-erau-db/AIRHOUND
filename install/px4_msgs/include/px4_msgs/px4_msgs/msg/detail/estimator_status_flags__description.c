// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorStatusFlags.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_status_flags__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorStatusFlags__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x60, 0x85, 0x26, 0x4c, 0x26, 0x38, 0x37, 0x83,
      0xc1, 0x3e, 0xc9, 0xea, 0x0f, 0x7a, 0xdf, 0xc7,
      0x81, 0xbd, 0x2c, 0x80, 0x88, 0xc2, 0x21, 0x8e,
      0x7d, 0x2f, 0x19, 0x4d, 0xb0, 0xc2, 0x75, 0x82,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorStatusFlags__TYPE_NAME[] = "px4_msgs/msg/EstimatorStatusFlags";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__control_status_changes[] = "control_status_changes";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_tilt_align[] = "cs_tilt_align";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_yaw_align[] = "cs_yaw_align";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_pos[] = "cs_gnss_pos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_opt_flow[] = "cs_opt_flow";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_hdg[] = "cs_mag_hdg";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_3d[] = "cs_mag_3d";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_dec[] = "cs_mag_dec";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_in_air[] = "cs_in_air";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_wind[] = "cs_wind";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_baro_hgt[] = "cs_baro_hgt";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_hgt[] = "cs_rng_hgt";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gps_hgt[] = "cs_gps_hgt";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_pos[] = "cs_ev_pos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_yaw[] = "cs_ev_yaw";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_hgt[] = "cs_ev_hgt";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fuse_beta[] = "cs_fuse_beta";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_field_disturbed[] = "cs_mag_field_disturbed";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fixed_wing[] = "cs_fixed_wing";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_fault[] = "cs_mag_fault";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fuse_aspd[] = "cs_fuse_aspd";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnd_effect[] = "cs_gnd_effect";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_stuck[] = "cs_rng_stuck";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_yaw[] = "cs_gnss_yaw";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_aligned_in_flight[] = "cs_mag_aligned_in_flight";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_vel[] = "cs_ev_vel";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_synthetic_mag_z[] = "cs_synthetic_mag_z";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_vehicle_at_rest[] = "cs_vehicle_at_rest";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_yaw_fault[] = "cs_gnss_yaw_fault";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_fault[] = "cs_rng_fault";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_inertial_dead_reckoning[] = "cs_inertial_dead_reckoning";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_wind_dead_reckoning[] = "cs_wind_dead_reckoning";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_kin_consistent[] = "cs_rng_kin_consistent";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fake_pos[] = "cs_fake_pos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fake_hgt[] = "cs_fake_hgt";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gravity_vector[] = "cs_gravity_vector";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag[] = "cs_mag";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_yaw_fault[] = "cs_ev_yaw_fault";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_heading_consistent[] = "cs_mag_heading_consistent";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_aux_gpos[] = "cs_aux_gpos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_terrain[] = "cs_rng_terrain";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_opt_flow_terrain[] = "cs_opt_flow_terrain";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_valid_fake_pos[] = "cs_valid_fake_pos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_constant_pos[] = "cs_constant_pos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_baro_fault[] = "cs_baro_fault";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_vel[] = "cs_gnss_vel";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_fault[] = "cs_gnss_fault";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_yaw_manual[] = "cs_yaw_manual";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_hgt_fault[] = "cs_gnss_hgt_fault";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fault_status_changes[] = "fault_status_changes";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_x[] = "fs_bad_mag_x";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_y[] = "fs_bad_mag_y";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_z[] = "fs_bad_mag_z";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_hdg[] = "fs_bad_hdg";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_decl[] = "fs_bad_mag_decl";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_airspeed[] = "fs_bad_airspeed";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_sideslip[] = "fs_bad_sideslip";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_optflow_x[] = "fs_bad_optflow_x";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_optflow_y[] = "fs_bad_optflow_y";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_acc_vertical[] = "fs_bad_acc_vertical";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_acc_clipping[] = "fs_bad_acc_clipping";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__innovation_fault_status_changes[] = "innovation_fault_status_changes";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_hor_vel[] = "reject_hor_vel";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_ver_vel[] = "reject_ver_vel";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_hor_pos[] = "reject_hor_pos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_ver_pos[] = "reject_ver_pos";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_yaw[] = "reject_yaw";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_airspeed[] = "reject_airspeed";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_sideslip[] = "reject_sideslip";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_hagl[] = "reject_hagl";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_optflow_x[] = "reject_optflow_x";
static char px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_optflow_y[] = "reject_optflow_y";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorStatusFlags__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__control_status_changes, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_tilt_align, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_yaw_align, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_pos, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_opt_flow, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_hdg, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_3d, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_dec, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_in_air, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_wind, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_baro_hgt, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_hgt, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gps_hgt, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_pos, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_yaw, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_hgt, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fuse_beta, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_field_disturbed, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fixed_wing, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_fault, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fuse_aspd, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnd_effect, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_stuck, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_yaw, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_aligned_in_flight, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_vel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_synthetic_mag_z, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_vehicle_at_rest, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_yaw_fault, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_fault, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_inertial_dead_reckoning, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_wind_dead_reckoning, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_kin_consistent, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fake_pos, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_fake_hgt, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gravity_vector, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_ev_yaw_fault, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_mag_heading_consistent, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_aux_gpos, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_rng_terrain, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_opt_flow_terrain, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_valid_fake_pos, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_constant_pos, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_baro_fault, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_vel, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_fault, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_yaw_manual, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__cs_gnss_hgt_fault, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fault_status_changes, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_x, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_y, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_z, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_hdg, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_mag_decl, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_airspeed, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_sideslip, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_optflow_x, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_optflow_y, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_acc_vertical, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__fs_bad_acc_clipping, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__innovation_fault_status_changes, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_hor_vel, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_ver_vel, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_hor_pos, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_ver_pos, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_yaw, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_airspeed, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_sideslip, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_hagl, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_optflow_x, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorStatusFlags__FIELD_NAME__reject_optflow_y, 16, 16},
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
px4_msgs__msg__EstimatorStatusFlags__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorStatusFlags__TYPE_NAME, 33, 33},
      {px4_msgs__msg__EstimatorStatusFlags__FIELDS, 74, 74},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                          # time since system start (microseconds)\n"
  "uint64 timestamp_sample                   # the timestamp of the raw data (microseconds)\n"
  "\n"
  "\n"
  "# filter control status\n"
  "uint32 control_status_changes # number of filter control status (cs) changes\n"
  "bool cs_tilt_align            #  0 - true if the filter tilt alignment is complete\n"
  "bool cs_yaw_align             #  1 - true if the filter yaw alignment is complete\n"
  "bool cs_gnss_pos              #  2 - true if GNSS position measurement fusion is intended\n"
  "bool cs_opt_flow              #  3 - true if optical flow measurements fusion is intended\n"
  "bool cs_mag_hdg               #  4 - true if a simple magnetic yaw heading fusion is intended\n"
  "bool cs_mag_3d                #  5 - true if 3-axis magnetometer measurement fusion is intended\n"
  "bool cs_mag_dec               #  6 - true if synthetic magnetic declination measurements fusion is intended\n"
  "bool cs_in_air                #  7 - true when the vehicle is airborne\n"
  "bool cs_wind                  #  8 - true when wind velocity is being estimated\n"
  "bool cs_baro_hgt              #  9 - true when baro data is being fused\n"
  "bool cs_rng_hgt               # 10 - true when range finder data is being fused for height aiding\n"
  "bool cs_gps_hgt               # 11 - true when GPS altitude is being fused\n"
  "bool cs_ev_pos                # 12 - true when local position data fusion from external vision is intended\n"
  "bool cs_ev_yaw                # 13 - true when yaw data from external vision measurements fusion is intended\n"
  "bool cs_ev_hgt                # 14 - true when height data from external vision measurements is being fused\n"
  "bool cs_fuse_beta             # 15 - true when synthetic sideslip measurements are being fused\n"
  "bool cs_mag_field_disturbed   # 16 - true when the mag field does not match the expected strength\n"
  "bool cs_fixed_wing            # 17 - true when the vehicle is operating as a fixed wing vehicle\n"
  "bool cs_mag_fault             # 18 - true when the magnetometer has been declared faulty and is no longer being used\n"
  "bool cs_fuse_aspd             # 19 - true when airspeed measurements are being fused\n"
  "bool cs_gnd_effect            # 20 - true when protection from ground effect induced static pressure rise is active\n"
  "bool cs_rng_stuck             # 21 - true when rng data wasn't ready for more than 10s and new rng values haven't changed enough\n"
  "bool cs_gnss_yaw              # 22 - true when yaw (not ground course) data fusion from a GPS receiver is intended\n"
  "bool cs_mag_aligned_in_flight # 23 - true when the in-flight mag field alignment has been completed\n"
  "bool cs_ev_vel                # 24 - true when local frame velocity data fusion from external vision measurements is intended\n"
  "bool cs_synthetic_mag_z       # 25 - true when we are using a synthesized measurement for the magnetometer Z component\n"
  "bool cs_vehicle_at_rest       # 26 - true when the vehicle is at rest\n"
  "bool cs_gnss_yaw_fault        # 27 - true when the GNSS heading has been declared faulty and is no longer being used\n"
  "bool cs_rng_fault             # 28 - true when the range finder has been declared faulty and is no longer being used\n"
  "bool cs_inertial_dead_reckoning # 29 - true if we are no longer fusing measurements that constrain horizontal velocity drift\n"
  "bool cs_wind_dead_reckoning     # 30 - true if we are navigationg reliant on wind relative measurements\n"
  "bool cs_rng_kin_consistent      # 31 - true when the range finder kinematic consistency check is passing\n"
  "bool cs_fake_pos                # 32 - true when fake position measurements are being fused\n"
  "bool cs_fake_hgt                # 33 - true when fake height measurements are being fused\n"
  "bool cs_gravity_vector          # 34 - true when gravity vector measurements are being fused\n"
  "bool cs_mag                     # 35 - true if 3-axis magnetometer measurement fusion (mag states only) is intended\n"
  "bool cs_ev_yaw_fault            # 36 - true when the EV heading has been declared faulty and is no longer being used\n"
  "bool cs_mag_heading_consistent  # 37 - true when the heading obtained from mag data is declared consistent with the filter\n"
  "bool cs_aux_gpos                # 38 - true if auxiliary global position measurement fusion is intended\n"
  "bool cs_rng_terrain             # 39 - true if we are fusing range finder data for terrain\n"
  "bool cs_opt_flow_terrain        # 40 - true if we are fusing flow data for terrain\n"
  "bool cs_valid_fake_pos          # 41 - true if a valid constant position is being fused\n"
  "bool cs_constant_pos            # 42 - true if the vehicle is at a constant position\n"
  "bool cs_baro_fault\\t        # 43 - true when the current baro has been declared faulty and is no longer being used\n"
  "bool cs_gnss_vel                # 44 - true if GNSS velocity measurement fusion is intended\n"
  "bool cs_gnss_fault              # 45 - true if GNSS true if GNSS measurements (lat, lon, vel) have been declared faulty\n"
  "bool cs_yaw_manual              # 46 - true if yaw has been set manually\n"
  "bool cs_gnss_hgt_fault          # 47 - true if GNSS true if GNSS measurements (alt) have been declared faulty\n"
  "\n"
  "# fault status\n"
  "uint32 fault_status_changes   # number of filter fault status (fs) changes\n"
  "bool fs_bad_mag_x             #  0 - true if the fusion of the magnetometer X-axis has encountered a numerical error\n"
  "bool fs_bad_mag_y             #  1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error\n"
  "bool fs_bad_mag_z             #  2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error\n"
  "bool fs_bad_hdg               #  3 - true if the fusion of the heading angle has encountered a numerical error\n"
  "bool fs_bad_mag_decl          #  4 - true if the fusion of the magnetic declination has encountered a numerical error\n"
  "bool fs_bad_airspeed          #  5 - true if fusion of the airspeed has encountered a numerical error\n"
  "bool fs_bad_sideslip          #  6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error\n"
  "bool fs_bad_optflow_x         #  7 - true if fusion of the optical flow X axis has encountered a numerical error\n"
  "bool fs_bad_optflow_y         #  8 - true if fusion of the optical flow Y axis has encountered a numerical error\n"
  "bool fs_bad_acc_vertical      # 10 - true if bad vertical accelerometer data has been detected\n"
  "bool fs_bad_acc_clipping      # 11 - true if delta velocity data contains clipping (asymmetric railing)\n"
  "\n"
  "\n"
  "# innovation test failures\n"
  "uint32 innovation_fault_status_changes    # number of innovation fault status (reject) changes\n"
  "bool reject_hor_vel                       #  0 - true if horizontal velocity observations have been rejected\n"
  "bool reject_ver_vel                       #  1 - true if vertical velocity observations have been rejected\n"
  "bool reject_hor_pos                       #  2 - true if horizontal position observations have been rejected\n"
  "bool reject_ver_pos                       #  3 - true if vertical position observations have been rejected\n"
  "bool reject_yaw                           #  7 - true if the yaw observation has been rejected\n"
  "bool reject_airspeed                      #  8 - true if the airspeed observation has been rejected\n"
  "bool reject_sideslip                      #  9 - true if the synthetic sideslip observation has been rejected\n"
  "bool reject_hagl                          # 10 - true if the height above ground observation has been rejected\n"
  "bool reject_optflow_x                     # 11 - true if the X optical flow observation has been rejected\n"
  "bool reject_optflow_y                     # 12 - true if the Y optical flow observation has been rejected";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorStatusFlags__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorStatusFlags__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 7568, 7568},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorStatusFlags__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorStatusFlags__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
