// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cpp_action_pkg:action/PathPlan.idl
// generated code does not contain a copyright notice

#ifndef CPP_ACTION_PKG__ACTION__DETAIL__PATH_PLAN__BUILDER_HPP_
#define CPP_ACTION_PKG__ACTION__DETAIL__PATH_PLAN__BUILDER_HPP_

#include "cpp_action_pkg/action/detail/path_plan__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_Goal_end_position
{
public:
  Init_PathPlan_Goal_end_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cpp_action_pkg::action::PathPlan_Goal end_position(::cpp_action_pkg::action::PathPlan_Goal::_end_position_type arg)
  {
    msg_.end_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_Goal>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_Goal_end_position();
}

}  // namespace cpp_action_pkg


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_Result_success
{
public:
  Init_PathPlan_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cpp_action_pkg::action::PathPlan_Result success(::cpp_action_pkg::action::PathPlan_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_Result>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_Result_success();
}

}  // namespace cpp_action_pkg


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_Feedback_current_position
{
public:
  Init_PathPlan_Feedback_current_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cpp_action_pkg::action::PathPlan_Feedback current_position(::cpp_action_pkg::action::PathPlan_Feedback::_current_position_type arg)
  {
    msg_.current_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_Feedback>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_Feedback_current_position();
}

}  // namespace cpp_action_pkg


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_SendGoal_Request_goal
{
public:
  explicit Init_PathPlan_SendGoal_Request_goal(::cpp_action_pkg::action::PathPlan_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::cpp_action_pkg::action::PathPlan_SendGoal_Request goal(::cpp_action_pkg::action::PathPlan_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_SendGoal_Request msg_;
};

class Init_PathPlan_SendGoal_Request_goal_id
{
public:
  Init_PathPlan_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlan_SendGoal_Request_goal goal_id(::cpp_action_pkg::action::PathPlan_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PathPlan_SendGoal_Request_goal(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_SendGoal_Request>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_SendGoal_Request_goal_id();
}

}  // namespace cpp_action_pkg


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_SendGoal_Response_stamp
{
public:
  explicit Init_PathPlan_SendGoal_Response_stamp(::cpp_action_pkg::action::PathPlan_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::cpp_action_pkg::action::PathPlan_SendGoal_Response stamp(::cpp_action_pkg::action::PathPlan_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_SendGoal_Response msg_;
};

class Init_PathPlan_SendGoal_Response_accepted
{
public:
  Init_PathPlan_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlan_SendGoal_Response_stamp accepted(::cpp_action_pkg::action::PathPlan_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PathPlan_SendGoal_Response_stamp(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_SendGoal_Response>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_SendGoal_Response_accepted();
}

}  // namespace cpp_action_pkg


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_GetResult_Request_goal_id
{
public:
  Init_PathPlan_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cpp_action_pkg::action::PathPlan_GetResult_Request goal_id(::cpp_action_pkg::action::PathPlan_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_GetResult_Request>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_GetResult_Request_goal_id();
}

}  // namespace cpp_action_pkg


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_GetResult_Response_result
{
public:
  explicit Init_PathPlan_GetResult_Response_result(::cpp_action_pkg::action::PathPlan_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::cpp_action_pkg::action::PathPlan_GetResult_Response result(::cpp_action_pkg::action::PathPlan_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_GetResult_Response msg_;
};

class Init_PathPlan_GetResult_Response_status
{
public:
  Init_PathPlan_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlan_GetResult_Response_result status(::cpp_action_pkg::action::PathPlan_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PathPlan_GetResult_Response_result(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_GetResult_Response>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_GetResult_Response_status();
}

}  // namespace cpp_action_pkg


namespace cpp_action_pkg
{

namespace action
{

namespace builder
{

class Init_PathPlan_FeedbackMessage_feedback
{
public:
  explicit Init_PathPlan_FeedbackMessage_feedback(::cpp_action_pkg::action::PathPlan_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::cpp_action_pkg::action::PathPlan_FeedbackMessage feedback(::cpp_action_pkg::action::PathPlan_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_FeedbackMessage msg_;
};

class Init_PathPlan_FeedbackMessage_goal_id
{
public:
  Init_PathPlan_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPlan_FeedbackMessage_feedback goal_id(::cpp_action_pkg::action::PathPlan_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PathPlan_FeedbackMessage_feedback(msg_);
  }

private:
  ::cpp_action_pkg::action::PathPlan_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_action_pkg::action::PathPlan_FeedbackMessage>()
{
  return cpp_action_pkg::action::builder::Init_PathPlan_FeedbackMessage_goal_id();
}

}  // namespace cpp_action_pkg

#endif  // CPP_ACTION_PKG__ACTION__DETAIL__PATH_PLAN__BUILDER_HPP_
