// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/battery_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__BatteryStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x92, 0xfd, 0x1f, 0x3a, 0xf4, 0x14, 0x93, 0x12,
      0x73, 0x69, 0x45, 0xb5, 0xa3, 0x53, 0x7b, 0x61,
      0xe5, 0x26, 0x8f, 0xca, 0x09, 0x0d, 0x27, 0xab,
      0xbb, 0x3b, 0x8b, 0xa2, 0x43, 0xa5, 0xdf, 0x8e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__BatteryStatus__TYPE_NAME[] = "px4_msgs/msg/BatteryStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__connected[] = "connected";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_v[] = "voltage_v";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__current_a[] = "current_a";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__current_average_a[] = "current_average_a";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__discharged_mah[] = "discharged_mah";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining[] = "remaining";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__scale[] = "scale";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__time_remaining_s[] = "time_remaining_s";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__temperature[] = "temperature";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__cell_count[] = "cell_count";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__source[] = "source";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__priority[] = "priority";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__capacity[] = "capacity";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__cycle_count[] = "cycle_count";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__average_time_to_empty[] = "average_time_to_empty";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__manufacture_date[] = "manufacture_date";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__state_of_health[] = "state_of_health";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__max_error[] = "max_error";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__id[] = "id";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__interface_error[] = "interface_error";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_cell_v[] = "voltage_cell_v";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__max_cell_voltage_delta[] = "max_cell_voltage_delta";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__is_powering_off[] = "is_powering_off";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__is_required[] = "is_required";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__warning[] = "warning";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__faults[] = "faults";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__full_charge_capacity_wh[] = "full_charge_capacity_wh";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining_capacity_wh[] = "remaining_capacity_wh";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__over_discharge_count[] = "over_discharge_count";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__nominal_voltage[] = "nominal_voltage";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__internal_resistance_estimate[] = "internal_resistance_estimate";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate[] = "ocv_estimate";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate_filtered[] = "ocv_estimate_filtered";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__volt_based_soc_estimate[] = "volt_based_soc_estimate";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_prediction[] = "voltage_prediction";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__prediction_error[] = "prediction_error";
static char px4_msgs__msg__BatteryStatus__FIELD_NAME__estimation_covariance_norm[] = "estimation_covariance_norm";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__BatteryStatus__FIELDS[] = {
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__connected, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_v, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__current_a, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__current_average_a, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__discharged_mah, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__scale, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__time_remaining_s, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__cell_count, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__source, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__priority, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__capacity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__cycle_count, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__average_time_to_empty, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__manufacture_date, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__state_of_health, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__max_error, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__interface_error, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_cell_v, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      14,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__max_cell_voltage_delta, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__is_powering_off, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__is_required, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__warning, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__faults, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__full_charge_capacity_wh, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__remaining_capacity_wh, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__over_discharge_count, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__nominal_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__internal_resistance_estimate, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__ocv_estimate_filtered, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__volt_based_soc_estimate, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__voltage_prediction, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__prediction_error, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__BatteryStatus__FIELD_NAME__estimation_covariance_norm, 26, 26},
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
px4_msgs__msg__BatteryStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__BatteryStatus__TYPE_NAME, 26, 26},
      {px4_msgs__msg__BatteryStatus__FIELDS, 38, 38},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Battery status\n"
  "#\n"
  "# Battery status information for up to 4 battery instances.\n"
  "# These are populated from power module and smart battery device drivers, and one battery updated from MAVLink.\n"
  "# Battery instance information is also logged and streamed in MAVLink telemetry.\n"
  "\n"
  "uint32 MESSAGE_VERSION = 1\n"
  "uint8 MAX_INSTANCES = 3\n"
  "\n"
  "uint64 timestamp  # [us] Time since system start\n"
  "\n"
  "bool connected             # Whether or not a battery is connected. For power modules this is based on a voltage threshold.\n"
  "float32 voltage_v          # [V] [@invalid 0] Battery voltage\n"
  "float32 current_a          # [A] [@invalid -1] Battery current\n"
  "float32 current_average_a  # [A] [@invalid -1] Battery current average (for FW average in level flight)\n"
  "float32 discharged_mah     # [mAh] [@invalid -1] Discharged amount\n"
  "float32 remaining          # [@range 0,1] [@invalid -1] Remaining capacity\n"
  "float32 scale              # [-] [@range 1,] [@invalid -1] Scaling factor to compensate for lower actuation power caused by voltage sag\n"
  "float32 time_remaining_s   # [s] [@invalid NaN] Predicted time remaining until battery is empty under previous averaged load\n"
  "float32 temperature        # [\\xc2\\xb0C] [@invalid NaN] Temperature of the battery\n"
  "uint8 cell_count           # [-] [@invalid 0] Number of cells\n"
  "\n"
  "\n"
  "uint8 source                   # [@enum SOURCE] Battery source\n"
  "uint8 SOURCE_POWER_MODULE = 0  # Power module\n"
  "uint8 SOURCE_EXTERNAL = 1      # External\n"
  "uint8 SOURCE_ESCS = 2          # ESCs\n"
  "\n"
  "uint8 priority                # [-] Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1\n"
  "uint16 capacity               # [mAh] Capacity of the battery when fully charged\n"
  "uint16 cycle_count            # [-] Number of discharge cycles the battery has experienced\n"
  "uint16 average_time_to_empty  # [minutes] Predicted remaining battery capacity based on the average rate of discharge\n"
  "uint16 manufacture_date       # [-] Manufacture date, part of serial number of the battery pack. Formatted as: Day + Month\\xc3\\x9732 + (Year\\xe2\\x80\\x931980)\\xc3\\x97512\n"
  "uint16 state_of_health        # [%] [@range 0, 100] State of health. FullChargeCapacity/DesignCapacity\n"
  "uint16 max_error              # [%] [@range 1, 100] Max error, expected margin of error in the state-of-charge calculation\n"
  "uint8 id                      # [-] ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed\n"
  "uint16 interface_error        # [-] Interface error counter\n"
  "\n"
  "float32[14] voltage_cell_v      # [V] [@invalid 0] Battery individual cell voltages\n"
  "float32 max_cell_voltage_delta  # [V] Max difference between individual cell voltages\n"
  "\n"
  "bool is_powering_off  # Power off event imminent indication, false if unknown\n"
  "bool is_required      # Set if the battery is explicitly required before arming\n"
  "\n"
  "uint8 warning                # [@enum WARNING STATE] Current battery warning\n"
  "uint8 WARNING_NONE = 0       # No battery low voltage warning active\n"
  "uint8 WARNING_LOW = 1        # Low voltage warning\n"
  "uint8 WARNING_CRITICAL = 2   # Critical voltage, return / abort immediately\n"
  "uint8 WARNING_EMERGENCY = 3  # Immediate landing required\n"
  "uint8 WARNING_FAILED = 4     # Battery has failed completely\n"
  "uint8 STATE_UNHEALTHY = 6    # Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field\n"
  "uint8 STATE_CHARGING = 7     # Battery is charging\n"
  "\n"
  "uint16 faults                          # [@enum FAULT] Smart battery supply status/fault flags (bitmask) for health indication\n"
  "uint8 FAULT_DEEP_DISCHARGE = 0         # Battery has deep discharged\n"
  "uint8 FAULT_SPIKES = 1                 # Voltage spikes\n"
  "uint8 FAULT_CELL_FAIL= 2               # One or more cells have failed\n"
  "uint8 FAULT_OVER_CURRENT = 3           # Over-current\n"
  "uint8 FAULT_OVER_TEMPERATURE = 4       # Over-temperature\n"
  "uint8 FAULT_UNDER_TEMPERATURE = 5      # Under-temperature fault\n"
  "uint8 FAULT_INCOMPATIBLE_VOLTAGE = 6   # Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage)\n"
  "uint8 FAULT_INCOMPATIBLE_FIRMWARE = 7  # Battery firmware is not compatible with current autopilot firmware\n"
  "uint8 FAULT_INCOMPATIBLE_MODEL = 8     # Battery model is not supported by the system\n"
  "uint8 FAULT_HARDWARE_FAILURE = 9       # Hardware problem\n"
  "uint8 FAULT_FAILED_TO_ARM = 10         # Battery had a problem while arming\n"
  "uint8 FAULT_COUNT = 11                 # Counter. Keep this as last element\n"
  "\n"
  "float32 full_charge_capacity_wh  # [Wh] Compensated battery capacity\n"
  "float32 remaining_capacity_wh    # [Wh] Compensated battery capacity remaining\n"
  "uint16 over_discharge_count      # [-] Number of battery overdischarge\n"
  "float32 nominal_voltage          # [V] Nominal voltage of the battery pack\n"
  "\n"
  "float32 internal_resistance_estimate  # [Ohm] Internal resistance per cell estimate\n"
  "float32 ocv_estimate                  # [V] Open circuit voltage estimate\n"
  "float32 ocv_estimate_filtered         # [V] Filtered open circuit voltage estimate\n"
  "float32 volt_based_soc_estimate       # [-] [@range 0, 1] Normalized volt based state of charge estimate\n"
  "float32 voltage_prediction            # [V] Predicted voltage\n"
  "float32 prediction_error              # [V] Prediction error\n"
  "float32 estimation_covariance_norm    # [-] Norm of the covariance matrix";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__BatteryStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__BatteryStatus__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 5279, 5279},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__BatteryStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__BatteryStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
