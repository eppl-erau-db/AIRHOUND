// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/OnboardComputerStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/onboard_computer_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__OnboardComputerStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x65, 0x0e, 0xed, 0x12, 0x47, 0x74, 0x71, 0x71,
      0xc4, 0x39, 0x7b, 0xe0, 0x74, 0xf3, 0x36, 0x04,
      0x89, 0xe5, 0xb0, 0xe7, 0xa3, 0xc2, 0x2b, 0x8d,
      0x6e, 0xbc, 0x4c, 0x3a, 0x52, 0x45, 0xac, 0x27,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__OnboardComputerStatus__TYPE_NAME[] = "px4_msgs/msg/OnboardComputerStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__uptime[] = "uptime";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__type[] = "type";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__cpu_cores[] = "cpu_cores";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__cpu_combined[] = "cpu_combined";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__gpu_cores[] = "gpu_cores";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__gpu_combined[] = "gpu_combined";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__temperature_board[] = "temperature_board";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__temperature_core[] = "temperature_core";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__fan_speed[] = "fan_speed";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__ram_usage[] = "ram_usage";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__ram_total[] = "ram_total";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__storage_type[] = "storage_type";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__storage_usage[] = "storage_usage";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__storage_total[] = "storage_total";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_type[] = "link_type";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_tx_rate[] = "link_tx_rate";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_rx_rate[] = "link_rx_rate";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_tx_max[] = "link_tx_max";
static char px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_rx_max[] = "link_rx_max";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__OnboardComputerStatus__FIELDS[] = {
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__uptime, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__cpu_cores, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      8,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__cpu_combined, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      10,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__gpu_cores, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__gpu_combined, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      10,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__temperature_board, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__temperature_core, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8_ARRAY,
      8,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__fan_speed, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__ram_usage, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__ram_total, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__storage_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__storage_usage, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__storage_total, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_tx_rate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_rx_rate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_tx_max, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__OnboardComputerStatus__FIELD_NAME__link_rx_max, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__OnboardComputerStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__OnboardComputerStatus__TYPE_NAME, 34, 34},
      {px4_msgs__msg__OnboardComputerStatus__FIELDS, 20, 20},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ONBOARD_COMPUTER_STATUS message data\n"
  "uint64 timestamp # [us] time since system start (microseconds)\n"
  "uint32 uptime\\t # [ms] time since system boot of the companion (milliseconds)\n"
  "\n"
  "uint8 type\\t # type of onboard computer 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.\n"
  "\n"
  "uint8[8] cpu_cores # CPU usage on the component in percent\n"
  "uint8[10] cpu_combined # Combined CPU usage as the last 10 slices of 100 MS\n"
  "uint8[4] gpu_cores # GPU usage on the component in percent\n"
  "uint8[10] gpu_combined # Combined GPU usage as the last 10 slices of 100 MS\n"
  "int8 temperature_board # [degC] Temperature of the board\n"
  "int8[8] temperature_core # [degC] Temperature of the CPU core\n"
  "int16[4] fan_speed # [rpm] Fan speeds\n"
  "uint32 ram_usage # [MB] Amount of used RAM on the component system\n"
  "uint32 ram_total # [MB] Total amount of RAM on the component system\n"
  "uint32[4] storage_type # Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable)\n"
  "uint32[4] storage_usage # [MB] Amount of used storage space on the component system\n"
  "uint32[4] storage_total # [MB] Total amount of storage space on the component system\n"
  "uint32[6] link_type # [Kb/s] Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh proprietary\n"
  "uint32[6] link_tx_rate # [Kb/s] Network traffic from the component system\n"
  "uint32[6] link_rx_rate # [Kb/s] Network traffic to the component system\n"
  "uint32[6] link_tx_max # [Kb/s] Network capacity from the component system\n"
  "uint32[6] link_rx_max # [Kb/s] Network capacity to the component system";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__OnboardComputerStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__OnboardComputerStatus__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1654, 1654},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__OnboardComputerStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__OnboardComputerStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
