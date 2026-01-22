// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x82, 0x8b, 0xdd, 0xbb, 0x7d, 0x4c, 0x2a, 0xa6,
      0xad, 0x93, 0x75, 0x79, 0x55, 0xf6, 0x89, 0x3b,
      0xe1, 0xec, 0x5d, 0x8f, 0x11, 0x88, 0x5e, 0xc7,
      0x71, 0x5b, 0xcd, 0xd7, 0x6b, 0x52, 0x26, 0xc9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleStatus__TYPE_NAME[] = "px4_msgs/msg/VehicleStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__armed_time[] = "armed_time";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__takeoff_time[] = "takeoff_time";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__arming_state[] = "arming_state";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__latest_arming_reason[] = "latest_arming_reason";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__latest_disarming_reason[] = "latest_disarming_reason";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__nav_state_timestamp[] = "nav_state_timestamp";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__nav_state_user_intention[] = "nav_state_user_intention";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__nav_state[] = "nav_state";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__executor_in_charge[] = "executor_in_charge";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__valid_nav_states_mask[] = "valid_nav_states_mask";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__can_set_nav_states_mask[] = "can_set_nav_states_mask";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__failure_detector_status[] = "failure_detector_status";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__hil_state[] = "hil_state";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__vehicle_type[] = "vehicle_type";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__failsafe[] = "failsafe";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__failsafe_and_user_took_over[] = "failsafe_and_user_took_over";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__failsafe_defer_state[] = "failsafe_defer_state";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__gcs_connection_lost[] = "gcs_connection_lost";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__gcs_connection_lost_counter[] = "gcs_connection_lost_counter";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__high_latency_data_link_lost[] = "high_latency_data_link_lost";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__is_vtol[] = "is_vtol";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__is_vtol_tailsitter[] = "is_vtol_tailsitter";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__in_transition_mode[] = "in_transition_mode";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__in_transition_to_fw[] = "in_transition_to_fw";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__system_type[] = "system_type";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__system_id[] = "system_id";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__component_id[] = "component_id";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__safety_button_available[] = "safety_button_available";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__safety_off[] = "safety_off";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__power_input_valid[] = "power_input_valid";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__usb_connected[] = "usb_connected";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__open_drone_id_system_present[] = "open_drone_id_system_present";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__open_drone_id_system_healthy[] = "open_drone_id_system_healthy";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__parachute_system_present[] = "parachute_system_present";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__parachute_system_healthy[] = "parachute_system_healthy";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__rc_calibration_in_progress[] = "rc_calibration_in_progress";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__calibration_enabled[] = "calibration_enabled";
static char px4_msgs__msg__VehicleStatus__FIELD_NAME__pre_flight_checks_pass[] = "pre_flight_checks_pass";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleStatus__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__armed_time, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__takeoff_time, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__arming_state, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__latest_arming_reason, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__latest_disarming_reason, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__nav_state_timestamp, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__nav_state_user_intention, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__nav_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__executor_in_charge, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__valid_nav_states_mask, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__can_set_nav_states_mask, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__failure_detector_status, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__hil_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__vehicle_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__failsafe, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__failsafe_and_user_took_over, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__failsafe_defer_state, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__gcs_connection_lost, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__gcs_connection_lost_counter, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__high_latency_data_link_lost, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__is_vtol, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__is_vtol_tailsitter, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__in_transition_mode, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__in_transition_to_fw, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__system_type, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__system_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__component_id, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__safety_button_available, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__safety_off, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__power_input_valid, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__usb_connected, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__open_drone_id_system_present, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__open_drone_id_system_healthy, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__parachute_system_present, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__parachute_system_healthy, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__rc_calibration_in_progress, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__calibration_enabled, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleStatus__FIELD_NAME__pre_flight_checks_pass, 22, 22},
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
px4_msgs__msg__VehicleStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleStatus__TYPE_NAME, 26, 26},
      {px4_msgs__msg__VehicleStatus__FIELDS, 39, 39},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Encodes the system state of the vehicle published by commander\n"
  "\n"
  "uint32 MESSAGE_VERSION = 1\n"
  "\n"
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "uint64 armed_time # Arming timestamp (microseconds)\n"
  "uint64 takeoff_time # Takeoff timestamp (microseconds)\n"
  "\n"
  "uint8 arming_state\n"
  "uint8 ARMING_STATE_DISARMED = 1\n"
  "uint8 ARMING_STATE_ARMED    = 2\n"
  "\n"
  "uint8 latest_arming_reason\n"
  "uint8 latest_disarming_reason\n"
  "uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0\n"
  "uint8 ARM_DISARM_REASON_STICK_GESTURE = 1\n"
  "uint8 ARM_DISARM_REASON_RC_SWITCH = 2\n"
  "uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3\n"
  "uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4\n"
  "uint8 ARM_DISARM_REASON_MISSION_START = 5\n"
  "uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6\n"
  "uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7\n"
  "uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8\n"
  "uint8 ARM_DISARM_REASON_KILL_SWITCH = 9\n"
  "uint8 ARM_DISARM_REASON_LOCKDOWN = 10\n"
  "uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11\n"
  "uint8 ARM_DISARM_REASON_SHUTDOWN = 12\n"
  "uint8 ARM_DISARM_REASON_UNIT_TEST = 13\n"
  "\n"
  "uint64 nav_state_timestamp # time when current nav_state activated\n"
  "\n"
  "uint8 nav_state_user_intention                  # Mode that the user selected (might be different from nav_state in a failsafe situation)\n"
  "\n"
  "uint8 nav_state                                 # Currently active mode\n"
  "uint8 NAVIGATION_STATE_MANUAL = 0               # Manual mode\n"
  "uint8 NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode\n"
  "uint8 NAVIGATION_STATE_POSCTL = 2               # Position control mode\n"
  "uint8 NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode\n"
  "uint8 NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode\n"
  "uint8 NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode\n"
  "uint8 NAVIGATION_STATE_POSITION_SLOW = 6\n"
  "uint8 NAVIGATION_STATE_FREE5 = 7\n"
  "uint8 NAVIGATION_STATE_ALTITUDE_CRUISE = 8      # Altitude with Cruise mode\n"
  "uint8 NAVIGATION_STATE_FREE3 = 9\n"
  "uint8 NAVIGATION_STATE_ACRO = 10                # Acro mode\n"
  "uint8 NAVIGATION_STATE_FREE2 = 11\n"
  "uint8 NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)\n"
  "uint8 NAVIGATION_STATE_TERMINATION = 13         # Termination mode\n"
  "uint8 NAVIGATION_STATE_OFFBOARD = 14\n"
  "uint8 NAVIGATION_STATE_STAB = 15                # Stabilized mode\n"
  "uint8 NAVIGATION_STATE_FREE1 = 16\n"
  "uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff\n"
  "uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land\n"
  "uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow\n"
  "uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target\n"
  "uint8 NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle\n"
  "uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter\n"
  "uint8 NAVIGATION_STATE_EXTERNAL1 = 23\n"
  "uint8 NAVIGATION_STATE_EXTERNAL2 = 24\n"
  "uint8 NAVIGATION_STATE_EXTERNAL3 = 25\n"
  "uint8 NAVIGATION_STATE_EXTERNAL4 = 26\n"
  "uint8 NAVIGATION_STATE_EXTERNAL5 = 27\n"
  "uint8 NAVIGATION_STATE_EXTERNAL6 = 28\n"
  "uint8 NAVIGATION_STATE_EXTERNAL7 = 29\n"
  "uint8 NAVIGATION_STATE_EXTERNAL8 = 30\n"
  "uint8 NAVIGATION_STATE_MAX = 31\n"
  "\n"
  "uint8 executor_in_charge                        # Current mode executor in charge (0=Autopilot)\n"
  "\n"
  "uint32 valid_nav_states_mask                    # Bitmask for all valid nav_state values\n"
  "uint32 can_set_nav_states_mask                  # Bitmask for all modes that a user can select\n"
  "\n"
  "# Bitmask of detected failures\n"
  "uint16 failure_detector_status\n"
  "uint16 FAILURE_NONE = 0\n"
  "uint16 FAILURE_ROLL = 1              # (1 << 0)\n"
  "uint16 FAILURE_PITCH = 2             # (1 << 1)\n"
  "uint16 FAILURE_ALT = 4               # (1 << 2)\n"
  "uint16 FAILURE_EXT = 8               # (1 << 3)\n"
  "uint16 FAILURE_ARM_ESC = 16          # (1 << 4)\n"
  "uint16 FAILURE_BATTERY = 32          # (1 << 5)\n"
  "uint16 FAILURE_IMBALANCED_PROP = 64  # (1 << 6)\n"
  "uint16 FAILURE_MOTOR = 128           # (1 << 7)\n"
  "\n"
  "uint8 hil_state\n"
  "uint8 HIL_STATE_OFF = 0\n"
  "uint8 HIL_STATE_ON = 1\n"
  "\n"
  "# Current vehicle locomotion method. A vehicle can have different methods (e.g. VTOL transitions from RW to FW method)\n"
  "uint8 vehicle_type\n"
  "uint8 VEHICLE_TYPE_UNSPECIFIED = 0\n"
  "uint8 VEHICLE_TYPE_ROTARY_WING = 1\n"
  "uint8 VEHICLE_TYPE_FIXED_WING = 2\n"
  "uint8 VEHICLE_TYPE_ROVER = 3\n"
  "\n"
  "uint8 FAILSAFE_DEFER_STATE_DISABLED = 0\n"
  "uint8 FAILSAFE_DEFER_STATE_ENABLED = 1\n"
  "uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # Failsafes deferred, but would trigger a failsafe\n"
  "\n"
  "bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)\n"
  "bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control\n"
  "uint8 failsafe_defer_state # one of FAILSAFE_DEFER_STATE_*\n"
  "\n"
  "# Link loss\n"
  "bool gcs_connection_lost              # datalink to GCS lost\n"
  "uint8 gcs_connection_lost_counter     # counts unique GCS connection lost events\n"
  "bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost\n"
  "\n"
  "# VTOL flags\n"
  "bool is_vtol             # True if the system is VTOL capable\n"
  "bool is_vtol_tailsitter  # True if the system performs a 90\\xc2\\xb0 pitch down rotation during transition from MC to FW\n"
  "bool in_transition_mode  # True if VTOL is doing a transition\n"
  "bool in_transition_to_fw # True if VTOL is doing a transition from MC to FW\n"
  "\n"
  "# MAVLink identification\n"
  "uint8 system_type  # system type, contains mavlink MAV_TYPE\n"
  "uint8 system_id\\t   # system id, contains MAVLink's system ID field\n"
  "uint8 component_id # subsystem / component id, contains MAVLink's component ID field\n"
  "\n"
  "bool safety_button_available # Set to true if a safety button is connected\n"
  "bool safety_off # Set to true if safety is off\n"
  "\n"
  "bool power_input_valid                            # set if input power is valid\n"
  "bool usb_connected                                # set to true (never cleared) once telemetry received from usb link\n"
  "\n"
  "bool open_drone_id_system_present\n"
  "bool open_drone_id_system_healthy\n"
  "\n"
  "bool parachute_system_present\n"
  "bool parachute_system_healthy\n"
  "\n"
  "bool rc_calibration_in_progress\n"
  "bool calibration_enabled\n"
  "\n"
  "bool pre_flight_checks_pass\\t\\t# true if all checks necessary to arm pass";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleStatus__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 5966, 5966},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
