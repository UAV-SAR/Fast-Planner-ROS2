// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/Serial.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__SERIAL__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__SERIAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SO3_CMD'.
enum
{
  quadrotor_msgs__msg__Serial__SO3_CMD = 115
};

/// Constant 'TRPY_CMD'.
enum
{
  quadrotor_msgs__msg__Serial__TRPY_CMD = 112
};

/// Constant 'STATUS_DATA'.
enum
{
  quadrotor_msgs__msg__Serial__STATUS_DATA = 99
};

/// Constant 'OUTPUT_DATA'.
enum
{
  quadrotor_msgs__msg__Serial__OUTPUT_DATA = 100
};

/// Constant 'PPR_OUTPUT_DATA'.
enum
{
  quadrotor_msgs__msg__Serial__PPR_OUTPUT_DATA = 116
};

/// Constant 'PPR_GAINS'.
enum
{
  quadrotor_msgs__msg__Serial__PPR_GAINS = 103
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Serial in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__Serial
{
  std_msgs__msg__Header header;
  uint8_t channel;
  uint8_t type;
  rosidl_runtime_c__uint8__Sequence data;
} quadrotor_msgs__msg__Serial;

// Struct for a sequence of quadrotor_msgs__msg__Serial.
typedef struct quadrotor_msgs__msg__Serial__Sequence
{
  quadrotor_msgs__msg__Serial * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__Serial__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__SERIAL__STRUCT_H_
