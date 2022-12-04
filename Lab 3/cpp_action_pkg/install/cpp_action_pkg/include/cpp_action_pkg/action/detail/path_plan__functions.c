// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cpp_action_pkg:action/PathPlan.idl
// generated code does not contain a copyright notice
#include "cpp_action_pkg/action/detail/path_plan__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `end_position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
cpp_action_pkg__action__PathPlan_Goal__init(cpp_action_pkg__action__PathPlan_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // end_position
  if (!geometry_msgs__msg__Point__init(&msg->end_position)) {
    cpp_action_pkg__action__PathPlan_Goal__fini(msg);
    return false;
  }
  return true;
}

void
cpp_action_pkg__action__PathPlan_Goal__fini(cpp_action_pkg__action__PathPlan_Goal * msg)
{
  if (!msg) {
    return;
  }
  // end_position
  geometry_msgs__msg__Point__fini(&msg->end_position);
}

bool
cpp_action_pkg__action__PathPlan_Goal__are_equal(const cpp_action_pkg__action__PathPlan_Goal * lhs, const cpp_action_pkg__action__PathPlan_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // end_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->end_position), &(rhs->end_position)))
  {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_Goal__copy(
  const cpp_action_pkg__action__PathPlan_Goal * input,
  cpp_action_pkg__action__PathPlan_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // end_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->end_position), &(output->end_position)))
  {
    return false;
  }
  return true;
}

cpp_action_pkg__action__PathPlan_Goal *
cpp_action_pkg__action__PathPlan_Goal__create()
{
  cpp_action_pkg__action__PathPlan_Goal * msg = (cpp_action_pkg__action__PathPlan_Goal *)malloc(sizeof(cpp_action_pkg__action__PathPlan_Goal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_Goal));
  bool success = cpp_action_pkg__action__PathPlan_Goal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_Goal__destroy(cpp_action_pkg__action__PathPlan_Goal * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_Goal__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_Goal__Sequence__init(cpp_action_pkg__action__PathPlan_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_Goal * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_Goal *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_Goal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_Goal__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_Goal__Sequence__fini(cpp_action_pkg__action__PathPlan_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_Goal__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_Goal__Sequence *
cpp_action_pkg__action__PathPlan_Goal__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_Goal__Sequence * array = (cpp_action_pkg__action__PathPlan_Goal__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_Goal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_Goal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_Goal__Sequence__destroy(cpp_action_pkg__action__PathPlan_Goal__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_Goal__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_Goal__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_Goal__Sequence * lhs, const cpp_action_pkg__action__PathPlan_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_Goal__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_Goal__Sequence * input,
  cpp_action_pkg__action__PathPlan_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_Goal);
    cpp_action_pkg__action__PathPlan_Goal * data =
      (cpp_action_pkg__action__PathPlan_Goal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_Goal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_Goal__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
cpp_action_pkg__action__PathPlan_Result__init(cpp_action_pkg__action__PathPlan_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
cpp_action_pkg__action__PathPlan_Result__fini(cpp_action_pkg__action__PathPlan_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
cpp_action_pkg__action__PathPlan_Result__are_equal(const cpp_action_pkg__action__PathPlan_Result * lhs, const cpp_action_pkg__action__PathPlan_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_Result__copy(
  const cpp_action_pkg__action__PathPlan_Result * input,
  cpp_action_pkg__action__PathPlan_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

cpp_action_pkg__action__PathPlan_Result *
cpp_action_pkg__action__PathPlan_Result__create()
{
  cpp_action_pkg__action__PathPlan_Result * msg = (cpp_action_pkg__action__PathPlan_Result *)malloc(sizeof(cpp_action_pkg__action__PathPlan_Result));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_Result));
  bool success = cpp_action_pkg__action__PathPlan_Result__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_Result__destroy(cpp_action_pkg__action__PathPlan_Result * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_Result__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_Result__Sequence__init(cpp_action_pkg__action__PathPlan_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_Result * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_Result *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_Result));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_Result__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_Result__Sequence__fini(cpp_action_pkg__action__PathPlan_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_Result__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_Result__Sequence *
cpp_action_pkg__action__PathPlan_Result__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_Result__Sequence * array = (cpp_action_pkg__action__PathPlan_Result__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_Result__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_Result__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_Result__Sequence__destroy(cpp_action_pkg__action__PathPlan_Result__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_Result__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_Result__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_Result__Sequence * lhs, const cpp_action_pkg__action__PathPlan_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_Result__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_Result__Sequence * input,
  cpp_action_pkg__action__PathPlan_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_Result);
    cpp_action_pkg__action__PathPlan_Result * data =
      (cpp_action_pkg__action__PathPlan_Result *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_Result__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_Result__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `current_position`
// already included above
// #include "geometry_msgs/msg/detail/point__functions.h"

bool
cpp_action_pkg__action__PathPlan_Feedback__init(cpp_action_pkg__action__PathPlan_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_position
  if (!geometry_msgs__msg__Point__init(&msg->current_position)) {
    cpp_action_pkg__action__PathPlan_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
cpp_action_pkg__action__PathPlan_Feedback__fini(cpp_action_pkg__action__PathPlan_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_position
  geometry_msgs__msg__Point__fini(&msg->current_position);
}

bool
cpp_action_pkg__action__PathPlan_Feedback__are_equal(const cpp_action_pkg__action__PathPlan_Feedback * lhs, const cpp_action_pkg__action__PathPlan_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->current_position), &(rhs->current_position)))
  {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_Feedback__copy(
  const cpp_action_pkg__action__PathPlan_Feedback * input,
  cpp_action_pkg__action__PathPlan_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->current_position), &(output->current_position)))
  {
    return false;
  }
  return true;
}

cpp_action_pkg__action__PathPlan_Feedback *
cpp_action_pkg__action__PathPlan_Feedback__create()
{
  cpp_action_pkg__action__PathPlan_Feedback * msg = (cpp_action_pkg__action__PathPlan_Feedback *)malloc(sizeof(cpp_action_pkg__action__PathPlan_Feedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_Feedback));
  bool success = cpp_action_pkg__action__PathPlan_Feedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_Feedback__destroy(cpp_action_pkg__action__PathPlan_Feedback * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_Feedback__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_Feedback__Sequence__init(cpp_action_pkg__action__PathPlan_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_Feedback * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_Feedback *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_Feedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_Feedback__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_Feedback__Sequence__fini(cpp_action_pkg__action__PathPlan_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_Feedback__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_Feedback__Sequence *
cpp_action_pkg__action__PathPlan_Feedback__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_Feedback__Sequence * array = (cpp_action_pkg__action__PathPlan_Feedback__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_Feedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_Feedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_Feedback__Sequence__destroy(cpp_action_pkg__action__PathPlan_Feedback__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_Feedback__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_Feedback__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_Feedback__Sequence * lhs, const cpp_action_pkg__action__PathPlan_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_Feedback__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_Feedback__Sequence * input,
  cpp_action_pkg__action__PathPlan_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_Feedback);
    cpp_action_pkg__action__PathPlan_Feedback * data =
      (cpp_action_pkg__action__PathPlan_Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_Feedback__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "cpp_action_pkg/action/detail/path_plan__functions.h"

bool
cpp_action_pkg__action__PathPlan_SendGoal_Request__init(cpp_action_pkg__action__PathPlan_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    cpp_action_pkg__action__PathPlan_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!cpp_action_pkg__action__PathPlan_Goal__init(&msg->goal)) {
    cpp_action_pkg__action__PathPlan_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Request__fini(cpp_action_pkg__action__PathPlan_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  cpp_action_pkg__action__PathPlan_Goal__fini(&msg->goal);
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Request__are_equal(const cpp_action_pkg__action__PathPlan_SendGoal_Request * lhs, const cpp_action_pkg__action__PathPlan_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!cpp_action_pkg__action__PathPlan_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Request__copy(
  const cpp_action_pkg__action__PathPlan_SendGoal_Request * input,
  cpp_action_pkg__action__PathPlan_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!cpp_action_pkg__action__PathPlan_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

cpp_action_pkg__action__PathPlan_SendGoal_Request *
cpp_action_pkg__action__PathPlan_SendGoal_Request__create()
{
  cpp_action_pkg__action__PathPlan_SendGoal_Request * msg = (cpp_action_pkg__action__PathPlan_SendGoal_Request *)malloc(sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Request));
  bool success = cpp_action_pkg__action__PathPlan_SendGoal_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Request__destroy(cpp_action_pkg__action__PathPlan_SendGoal_Request * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_SendGoal_Request__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__init(cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_SendGoal_Request * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_SendGoal_Request *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_SendGoal_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__fini(cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_SendGoal_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence *
cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * array = (cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__destroy(cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * lhs, const cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * input,
  cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Request);
    cpp_action_pkg__action__PathPlan_SendGoal_Request * data =
      (cpp_action_pkg__action__PathPlan_SendGoal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_SendGoal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_SendGoal_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
cpp_action_pkg__action__PathPlan_SendGoal_Response__init(cpp_action_pkg__action__PathPlan_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    cpp_action_pkg__action__PathPlan_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Response__fini(cpp_action_pkg__action__PathPlan_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Response__are_equal(const cpp_action_pkg__action__PathPlan_SendGoal_Response * lhs, const cpp_action_pkg__action__PathPlan_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Response__copy(
  const cpp_action_pkg__action__PathPlan_SendGoal_Response * input,
  cpp_action_pkg__action__PathPlan_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

cpp_action_pkg__action__PathPlan_SendGoal_Response *
cpp_action_pkg__action__PathPlan_SendGoal_Response__create()
{
  cpp_action_pkg__action__PathPlan_SendGoal_Response * msg = (cpp_action_pkg__action__PathPlan_SendGoal_Response *)malloc(sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Response));
  bool success = cpp_action_pkg__action__PathPlan_SendGoal_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Response__destroy(cpp_action_pkg__action__PathPlan_SendGoal_Response * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_SendGoal_Response__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__init(cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_SendGoal_Response * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_SendGoal_Response *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_SendGoal_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__fini(cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_SendGoal_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence *
cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * array = (cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__destroy(cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * lhs, const cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * input,
  cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_SendGoal_Response);
    cpp_action_pkg__action__PathPlan_SendGoal_Response * data =
      (cpp_action_pkg__action__PathPlan_SendGoal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_SendGoal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_SendGoal_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
cpp_action_pkg__action__PathPlan_GetResult_Request__init(cpp_action_pkg__action__PathPlan_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    cpp_action_pkg__action__PathPlan_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Request__fini(cpp_action_pkg__action__PathPlan_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Request__are_equal(const cpp_action_pkg__action__PathPlan_GetResult_Request * lhs, const cpp_action_pkg__action__PathPlan_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Request__copy(
  const cpp_action_pkg__action__PathPlan_GetResult_Request * input,
  cpp_action_pkg__action__PathPlan_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

cpp_action_pkg__action__PathPlan_GetResult_Request *
cpp_action_pkg__action__PathPlan_GetResult_Request__create()
{
  cpp_action_pkg__action__PathPlan_GetResult_Request * msg = (cpp_action_pkg__action__PathPlan_GetResult_Request *)malloc(sizeof(cpp_action_pkg__action__PathPlan_GetResult_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_GetResult_Request));
  bool success = cpp_action_pkg__action__PathPlan_GetResult_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Request__destroy(cpp_action_pkg__action__PathPlan_GetResult_Request * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_GetResult_Request__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__init(cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_GetResult_Request * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_GetResult_Request *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_GetResult_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_GetResult_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__fini(cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_GetResult_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence *
cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * array = (cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__destroy(cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * lhs, const cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * input,
  cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_GetResult_Request);
    cpp_action_pkg__action__PathPlan_GetResult_Request * data =
      (cpp_action_pkg__action__PathPlan_GetResult_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_GetResult_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_GetResult_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "cpp_action_pkg/action/detail/path_plan__functions.h"

bool
cpp_action_pkg__action__PathPlan_GetResult_Response__init(cpp_action_pkg__action__PathPlan_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!cpp_action_pkg__action__PathPlan_Result__init(&msg->result)) {
    cpp_action_pkg__action__PathPlan_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Response__fini(cpp_action_pkg__action__PathPlan_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  cpp_action_pkg__action__PathPlan_Result__fini(&msg->result);
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Response__are_equal(const cpp_action_pkg__action__PathPlan_GetResult_Response * lhs, const cpp_action_pkg__action__PathPlan_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!cpp_action_pkg__action__PathPlan_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Response__copy(
  const cpp_action_pkg__action__PathPlan_GetResult_Response * input,
  cpp_action_pkg__action__PathPlan_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!cpp_action_pkg__action__PathPlan_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

cpp_action_pkg__action__PathPlan_GetResult_Response *
cpp_action_pkg__action__PathPlan_GetResult_Response__create()
{
  cpp_action_pkg__action__PathPlan_GetResult_Response * msg = (cpp_action_pkg__action__PathPlan_GetResult_Response *)malloc(sizeof(cpp_action_pkg__action__PathPlan_GetResult_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_GetResult_Response));
  bool success = cpp_action_pkg__action__PathPlan_GetResult_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Response__destroy(cpp_action_pkg__action__PathPlan_GetResult_Response * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_GetResult_Response__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__init(cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_GetResult_Response * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_GetResult_Response *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_GetResult_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_GetResult_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__fini(cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_GetResult_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence *
cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * array = (cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__destroy(cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * lhs, const cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * input,
  cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_GetResult_Response);
    cpp_action_pkg__action__PathPlan_GetResult_Response * data =
      (cpp_action_pkg__action__PathPlan_GetResult_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_GetResult_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_GetResult_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "cpp_action_pkg/action/detail/path_plan__functions.h"

bool
cpp_action_pkg__action__PathPlan_FeedbackMessage__init(cpp_action_pkg__action__PathPlan_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    cpp_action_pkg__action__PathPlan_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!cpp_action_pkg__action__PathPlan_Feedback__init(&msg->feedback)) {
    cpp_action_pkg__action__PathPlan_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
cpp_action_pkg__action__PathPlan_FeedbackMessage__fini(cpp_action_pkg__action__PathPlan_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  cpp_action_pkg__action__PathPlan_Feedback__fini(&msg->feedback);
}

bool
cpp_action_pkg__action__PathPlan_FeedbackMessage__are_equal(const cpp_action_pkg__action__PathPlan_FeedbackMessage * lhs, const cpp_action_pkg__action__PathPlan_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!cpp_action_pkg__action__PathPlan_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_FeedbackMessage__copy(
  const cpp_action_pkg__action__PathPlan_FeedbackMessage * input,
  cpp_action_pkg__action__PathPlan_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!cpp_action_pkg__action__PathPlan_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

cpp_action_pkg__action__PathPlan_FeedbackMessage *
cpp_action_pkg__action__PathPlan_FeedbackMessage__create()
{
  cpp_action_pkg__action__PathPlan_FeedbackMessage * msg = (cpp_action_pkg__action__PathPlan_FeedbackMessage *)malloc(sizeof(cpp_action_pkg__action__PathPlan_FeedbackMessage));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_action_pkg__action__PathPlan_FeedbackMessage));
  bool success = cpp_action_pkg__action__PathPlan_FeedbackMessage__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
cpp_action_pkg__action__PathPlan_FeedbackMessage__destroy(cpp_action_pkg__action__PathPlan_FeedbackMessage * msg)
{
  if (msg) {
    cpp_action_pkg__action__PathPlan_FeedbackMessage__fini(msg);
  }
  free(msg);
}


bool
cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__init(cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  cpp_action_pkg__action__PathPlan_FeedbackMessage * data = NULL;
  if (size) {
    data = (cpp_action_pkg__action__PathPlan_FeedbackMessage *)calloc(size, sizeof(cpp_action_pkg__action__PathPlan_FeedbackMessage));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_action_pkg__action__PathPlan_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_action_pkg__action__PathPlan_FeedbackMessage__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__fini(cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cpp_action_pkg__action__PathPlan_FeedbackMessage__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence *
cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__create(size_t size)
{
  cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * array = (cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence *)malloc(sizeof(cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__destroy(cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * array)
{
  if (array) {
    cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__fini(array);
  }
  free(array);
}

bool
cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__are_equal(const cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * lhs, const cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence__copy(
  const cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * input,
  cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_action_pkg__action__PathPlan_FeedbackMessage);
    cpp_action_pkg__action__PathPlan_FeedbackMessage * data =
      (cpp_action_pkg__action__PathPlan_FeedbackMessage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_action_pkg__action__PathPlan_FeedbackMessage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          cpp_action_pkg__action__PathPlan_FeedbackMessage__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_action_pkg__action__PathPlan_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
