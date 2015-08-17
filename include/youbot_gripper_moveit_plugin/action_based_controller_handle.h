
#ifndef MOVEIT_PLUGINS_ACTION_BASED_CONTROLLER_HANDLE
#define MOVEIT_PLUGINS_ACTION_BASED_CONTROLLER_HANDLE

#include <moveit/controller_manager/controller_manager.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/macros/class_forward.h>

namespace youbot_gripper_moveit_plugin
{

/*
 * This exist solely to inject addJoint/getJoints into base non-templated class.
 */
class ActionBasedControllerHandleBase : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ActionBasedControllerHandleBase(const std::string &name) :
    moveit_controller_manager::MoveItControllerHandle(name)
  {
  }

  virtual void addJoint(const std::string &name) = 0;
  virtual void getJoints(std::vector<std::string> &joints) = 0;
};

MOVEIT_CLASS_FORWARD(ActionBasedControllerHandleBase);


/*
 * This is a simple base class, which handles all of the action creation/etc
 */
template<typename T>
class ActionBasedControllerHandle : public ActionBasedControllerHandleBase
{

public:
  ActionBasedControllerHandle(const std::string &name, const std::string &ns) :
    ActionBasedControllerHandleBase(name),
    namespace_(ns),
    done_(true)
  {
    controller_action_client_.reset(new actionlib::SimpleActionClient<T>(getActionName(), true));
    unsigned int attempts = 0;
    while (ros::ok() && !controller_action_client_->waitForServer(ros::Duration(5.0)) && ++attempts < 3)
      ROS_INFO_STREAM("YoubotGripperMoveitPlugin: Waiting for " << getActionName() << " to come up");

    if (!controller_action_client_->isServerConnected())
    {
      ROS_ERROR_STREAM("YoubotGripperMoveitPlugin: Action client not connected: " << getActionName());
      controller_action_client_.reset();
    }

    last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  bool isConnected() const
  {
    return controller_action_client_;
  }

  virtual bool cancelExecution()
  {
    if (!controller_action_client_)
      return false;
    if (!done_)
    {
      ROS_INFO_STREAM("YoubotGripperMoveitPlugin: Cancelling execution for " << name_);
      controller_action_client_->cancelGoal();
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      done_ = true;
    }
    return true;
  }

  virtual bool waitForExecution(const ros::Duration &timeout = ros::Duration(0))
  {
    if (controller_action_client_ && !done_)
      return controller_action_client_->waitForResult(timeout);
    return true;
  }

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
  {
    return last_exec_;
  }

  virtual void addJoint(const std::string &name)
  {
    joints_.push_back(name);
  }

  virtual void getJoints(std::vector<std::string> &joints)
  {
    joints = joints_;
  }

protected:

  std::string getActionName(void) const
  {
    if (namespace_.empty())
      return name_;
    else
      return name_ +"/" + namespace_;
  }

  void finishControllerExecution(const actionlib::SimpleClientGoalState& state)
  {
    ROS_DEBUG_STREAM("YoubotGripperMoveitPlugin: Controller " << name_ << " is done with state " << state.toString() << ": " << state.getText());
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else
      if (state == actionlib::SimpleClientGoalState::ABORTED)
        last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
      else
        if (state == actionlib::SimpleClientGoalState::PREEMPTED)
          last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
        else
          last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    done_ = true;
  }

  /* execution status */
  moveit_controller_manager::ExecutionStatus last_exec_;
  bool done_;

  /* the controller namespace, for instance, topics will map to name/ns/goal, name/ns/result, etc */
  std::string namespace_;

  /* the joints controlled by this controller */
  std::vector<std::string> joints_;

  /* action client */
  boost::shared_ptr<actionlib::SimpleActionClient<T> > controller_action_client_;
};


} // end namespace youbot_gripper_moveit_plugin

#endif // MOVEIT_PLUGINS_ACTION_BASED_CONTROLLER_HANDLE
