// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:srv/VehicleCommand.idl
// generated code does not contain a copyright notice

#include "px4_msgs/srv/detail/vehicle_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__srv__VehicleCommand__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa6, 0x99, 0x78, 0xef, 0x4f, 0xdb, 0x14, 0xa0,
      0xfe, 0x16, 0x04, 0xac, 0x42, 0xae, 0xeb, 0xee,
      0x82, 0x2b, 0x64, 0xe8, 0x38, 0x2d, 0xc3, 0x0a,
      0xf4, 0xea, 0xc3, 0x17, 0xfb, 0xe0, 0x45, 0x03,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__srv__VehicleCommand_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf5, 0xe0, 0x48, 0x3c, 0x8c, 0x12, 0x1b, 0x6a,
      0x57, 0x85, 0xb4, 0x67, 0xfb, 0x26, 0x28, 0xb0,
      0x24, 0x35, 0x8a, 0xa6, 0x35, 0xd8, 0x36, 0x12,
      0x3b, 0x32, 0x6d, 0xb2, 0xc7, 0xd9, 0xdb, 0x7c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__srv__VehicleCommand_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa8, 0xbd, 0x91, 0x85, 0x9d, 0xa7, 0x2f, 0x95,
      0x9a, 0xed, 0xd8, 0x83, 0x39, 0x60, 0x56, 0x7c,
      0xea, 0x6f, 0x69, 0x88, 0x0a, 0xeb, 0xe7, 0x33,
      0x19, 0x6c, 0x5d, 0x64, 0x86, 0x7d, 0x1b, 0x86,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__srv__VehicleCommand_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x82, 0xa4, 0x22, 0x35, 0x30, 0xdd, 0x1c, 0x7a,
      0xba, 0x17, 0x22, 0x85, 0x8b, 0xd2, 0x54, 0x96,
      0xb5, 0x60, 0x57, 0xdd, 0x16, 0x66, 0x57, 0xf8,
      0xae, 0xec, 0xbf, 0x56, 0x27, 0x00, 0x1e, 0x8e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "px4_msgs/msg/detail/vehicle_command__functions.h"
#include "px4_msgs/msg/detail/vehicle_command_ack__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t px4_msgs__msg__VehicleCommand__EXPECTED_HASH = {1, {
    0x1f, 0x0a, 0xb5, 0x37, 0x80, 0x7e, 0x2e, 0x20,
    0xe0, 0x48, 0x26, 0x63, 0x3e, 0xa2, 0xc5, 0x2e,
    0x54, 0x31, 0xbe, 0xa8, 0xeb, 0x01, 0x87, 0x15,
    0x2f, 0xf2, 0x3b, 0xd0, 0x7f, 0x1d, 0xd3, 0x0b,
  }};
static const rosidl_type_hash_t px4_msgs__msg__VehicleCommandAck__EXPECTED_HASH = {1, {
    0x36, 0x03, 0x8d, 0x69, 0xf9, 0xdb, 0x90, 0x98,
    0x0f, 0xb8, 0xfe, 0x15, 0x4f, 0x27, 0xf2, 0x2d,
    0xff, 0xed, 0x0c, 0x35, 0xe5, 0x30, 0xb3, 0x24,
    0x13, 0xcf, 0x04, 0x56, 0xaa, 0x8a, 0x29, 0xe2,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char px4_msgs__srv__VehicleCommand__TYPE_NAME[] = "px4_msgs/srv/VehicleCommand";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char px4_msgs__msg__VehicleCommand__TYPE_NAME[] = "px4_msgs/msg/VehicleCommand";
static char px4_msgs__msg__VehicleCommandAck__TYPE_NAME[] = "px4_msgs/msg/VehicleCommandAck";
static char px4_msgs__srv__VehicleCommand_Event__TYPE_NAME[] = "px4_msgs/srv/VehicleCommand_Event";
static char px4_msgs__srv__VehicleCommand_Request__TYPE_NAME[] = "px4_msgs/srv/VehicleCommand_Request";
static char px4_msgs__srv__VehicleCommand_Response__TYPE_NAME[] = "px4_msgs/srv/VehicleCommand_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char px4_msgs__srv__VehicleCommand__FIELD_NAME__request_message[] = "request_message";
static char px4_msgs__srv__VehicleCommand__FIELD_NAME__response_message[] = "response_message";
static char px4_msgs__srv__VehicleCommand__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field px4_msgs__srv__VehicleCommand__FIELDS[] = {
  {
    {px4_msgs__srv__VehicleCommand__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__srv__VehicleCommand_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__srv__VehicleCommand_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__srv__VehicleCommand_Event__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription px4_msgs__srv__VehicleCommand__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleCommand__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleCommandAck__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand_Event__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__srv__VehicleCommand__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__srv__VehicleCommand__TYPE_NAME, 27, 27},
      {px4_msgs__srv__VehicleCommand__FIELDS, 3, 3},
    },
    {px4_msgs__srv__VehicleCommand__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&px4_msgs__msg__VehicleCommand__EXPECTED_HASH, px4_msgs__msg__VehicleCommand__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = px4_msgs__msg__VehicleCommand__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&px4_msgs__msg__VehicleCommandAck__EXPECTED_HASH, px4_msgs__msg__VehicleCommandAck__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = px4_msgs__msg__VehicleCommandAck__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = px4_msgs__srv__VehicleCommand_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = px4_msgs__srv__VehicleCommand_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = px4_msgs__srv__VehicleCommand_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char px4_msgs__srv__VehicleCommand_Request__FIELD_NAME__request[] = "request";

static rosidl_runtime_c__type_description__Field px4_msgs__srv__VehicleCommand_Request__FIELDS[] = {
  {
    {px4_msgs__srv__VehicleCommand_Request__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__msg__VehicleCommand__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription px4_msgs__srv__VehicleCommand_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {px4_msgs__msg__VehicleCommand__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__srv__VehicleCommand_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__srv__VehicleCommand_Request__TYPE_NAME, 35, 35},
      {px4_msgs__srv__VehicleCommand_Request__FIELDS, 1, 1},
    },
    {px4_msgs__srv__VehicleCommand_Request__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&px4_msgs__msg__VehicleCommand__EXPECTED_HASH, px4_msgs__msg__VehicleCommand__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = px4_msgs__msg__VehicleCommand__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char px4_msgs__srv__VehicleCommand_Response__FIELD_NAME__reply[] = "reply";

static rosidl_runtime_c__type_description__Field px4_msgs__srv__VehicleCommand_Response__FIELDS[] = {
  {
    {px4_msgs__srv__VehicleCommand_Response__FIELD_NAME__reply, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__msg__VehicleCommandAck__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription px4_msgs__srv__VehicleCommand_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {px4_msgs__msg__VehicleCommandAck__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__srv__VehicleCommand_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__srv__VehicleCommand_Response__TYPE_NAME, 36, 36},
      {px4_msgs__srv__VehicleCommand_Response__FIELDS, 1, 1},
    },
    {px4_msgs__srv__VehicleCommand_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&px4_msgs__msg__VehicleCommandAck__EXPECTED_HASH, px4_msgs__msg__VehicleCommandAck__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = px4_msgs__msg__VehicleCommandAck__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char px4_msgs__srv__VehicleCommand_Event__FIELD_NAME__info[] = "info";
static char px4_msgs__srv__VehicleCommand_Event__FIELD_NAME__request[] = "request";
static char px4_msgs__srv__VehicleCommand_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field px4_msgs__srv__VehicleCommand_Event__FIELDS[] = {
  {
    {px4_msgs__srv__VehicleCommand_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {px4_msgs__srv__VehicleCommand_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {px4_msgs__srv__VehicleCommand_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription px4_msgs__srv__VehicleCommand_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleCommand__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleCommandAck__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {px4_msgs__srv__VehicleCommand_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__srv__VehicleCommand_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__srv__VehicleCommand_Event__TYPE_NAME, 33, 33},
      {px4_msgs__srv__VehicleCommand_Event__FIELDS, 3, 3},
    },
    {px4_msgs__srv__VehicleCommand_Event__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&px4_msgs__msg__VehicleCommand__EXPECTED_HASH, px4_msgs__msg__VehicleCommand__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = px4_msgs__msg__VehicleCommand__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&px4_msgs__msg__VehicleCommandAck__EXPECTED_HASH, px4_msgs__msg__VehicleCommandAck__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = px4_msgs__msg__VehicleCommandAck__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = px4_msgs__srv__VehicleCommand_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = px4_msgs__srv__VehicleCommand_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "VehicleCommand request\n"
  "---\n"
  "VehicleCommandAck reply";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__srv__VehicleCommand__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__srv__VehicleCommand__TYPE_NAME, 27, 27},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 51, 51},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__srv__VehicleCommand_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__srv__VehicleCommand_Request__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__srv__VehicleCommand_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__srv__VehicleCommand_Response__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__srv__VehicleCommand_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__srv__VehicleCommand_Event__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__srv__VehicleCommand__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__srv__VehicleCommand__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *px4_msgs__msg__VehicleCommand__get_individual_type_description_source(NULL);
    sources[3] = *px4_msgs__msg__VehicleCommandAck__get_individual_type_description_source(NULL);
    sources[4] = *px4_msgs__srv__VehicleCommand_Event__get_individual_type_description_source(NULL);
    sources[5] = *px4_msgs__srv__VehicleCommand_Request__get_individual_type_description_source(NULL);
    sources[6] = *px4_msgs__srv__VehicleCommand_Response__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__srv__VehicleCommand_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__srv__VehicleCommand_Request__get_individual_type_description_source(NULL),
    sources[1] = *px4_msgs__msg__VehicleCommand__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__srv__VehicleCommand_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__srv__VehicleCommand_Response__get_individual_type_description_source(NULL),
    sources[1] = *px4_msgs__msg__VehicleCommandAck__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__srv__VehicleCommand_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__srv__VehicleCommand_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *px4_msgs__msg__VehicleCommand__get_individual_type_description_source(NULL);
    sources[3] = *px4_msgs__msg__VehicleCommandAck__get_individual_type_description_source(NULL);
    sources[4] = *px4_msgs__srv__VehicleCommand_Request__get_individual_type_description_source(NULL);
    sources[5] = *px4_msgs__srv__VehicleCommand_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
