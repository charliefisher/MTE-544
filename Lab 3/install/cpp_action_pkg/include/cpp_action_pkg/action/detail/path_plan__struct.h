// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cpp_action_pkg:action/PathPlan.idl
// generated code does not contain a copyright notice

#ifndef CPP_ACTION_PKG__ACTION__DETAIL__PATH_PLAN__STRUCT_H_
#define CPP_ACTION_PKG__ACTION__DETAIL__PATH_PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'end_position'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_Goal
{
  geometry_msgs__msg__Point end_position;
} cpp_action_pkg__action__PathPlan_Goal;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_Goal.
typedef struct cpp_action_pkg__action__PathPlan_Goal__Sequence
{
  cpp_action_pkg__action__PathPlan_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_Result
{
  bool success;
} cpp_action_pkg__action__PathPlan_Result;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_Result.
typedef struct cpp_action_pkg__action__PathPlan_Result__Sequence
{
  cpp_action_pkg__action__PathPlan_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_position'
// already included above
// #include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_Feedback
{
  geometry_msgs__msg__Point current_position;
} cpp_action_pkg__action__PathPlan_Feedback;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_Feedback.
typedef struct cpp_action_pkg__action__PathPlan_Feedback__Sequence
{
  cpp_action_pkg__action__PathPlan_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "cpp_action_pkg/action/detail/path_plan__struct.h"

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  cpp_action_pkg__action__PathPlan_Goal goal;
} cpp_action_pkg__action__PathPlan_SendGoal_Request;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_SendGoal_Request.
typedef struct cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence
{
  cpp_action_pkg__action__PathPlan_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} cpp_action_pkg__action__PathPlan_SendGoal_Response;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_SendGoal_Response.
typedef struct cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence
{
  cpp_action_pkg__action__PathPlan_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} cpp_action_pkg__action__PathPlan_GetResult_Request;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_GetResult_Request.
typedef struct cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence
{
  cpp_action_pkg__action__PathPlan_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "cpp_action_pkg/action/detail/path_plan__struct.h"

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_GetResult_Response
{
  int8_t status;
  cpp_action_pkg__action__PathPlan_Result result;
} cpp_action_pkg__action__PathPlan_GetResult_Response;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_GetResult_Response.
typedef struct cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence
{
  cpp_action_pkg__action__PathPlan_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "cpp_action_pkg/action/detail/path_plan__struct.h"

// Struct defined in action/PathPlan in the package cpp_action_pkg.
typedef struct cpp_action_pkg__action__PathPlan_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  cpp_action_pkg__action__PathPlan_Feedback feedback;
} cpp_action_pkg__action__PathPlan_FeedbackMessage;

// Struct for a sequence of cpp_action_pkg__action__PathPlan_FeedbackMessage.
typedef struct cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence
{
  cpp_action_pkg__action__PathPlan_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_action_pkg__action__PathPlan_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CPP_ACTION_PKG__ACTION__DETAIL__PATH_PLAN__STRUCT_H_
