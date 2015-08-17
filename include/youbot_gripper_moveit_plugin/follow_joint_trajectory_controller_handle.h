
#ifndef MOVEIT_PLUGINS_FOLLOW_TRAJECTORY_CONTROLLER_HANDLE
#define MOVEIT_PLUGINS_FOLLOW_TRAJECTORY_CONTROLLER_HANDLE

#include <youbot_gripper_moveit_plugin/action_based_controller_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace youbot_gripper_moveit_plugin
{

/*
 * This is generally used for arms, but could also be used for multi-dof hands,
 *   or anything using a control_mgs/FollowJointTrajectoryAction.
 */
class FollowJointTrajectoryControllerHandle : public ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>
{
public:

  FollowJointTrajectoryControllerHandle(const std::string &name, const std::string &action_ns) :
    ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>(name, action_ns)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_DEBUG_STREAM("FollowJointTrajectoryController: new trajectory to " << name_);

    if (!controller_action_client_)
      return false;

    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_WARN("FollowJointTrajectoryController: %s cannot execute multi-dof trajectories.", name_.c_str());
    }

    if (done_)
      ROS_DEBUG_STREAM("FollowJointTrajectoryController: sending trajectory to " << name_);
    else
      ROS_DEBUG_STREAM("FollowJointTrajectoryController: sending continuation for the currently executed trajectory to " << name_);

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory.joint_trajectory;
    controller_action_client_->sendGoal(goal,
                    boost::bind(&FollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
                    boost::bind(&FollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
                    boost::bind(&FollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

protected:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::FollowJointTrajectoryResultConstPtr& result)
  {
    // Output custom error message for FollowJointTrajectoryResult if necessary
    switch( result->error_code )
    {
      case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL:
        ROS_WARN_STREAM("Controller " << name_ << " failed with error code INVALID_GOAL");
        break;
      case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS:
        ROS_WARN_STREAM("Controller " << name_ << " failed with error code INVALID_JOINTS");
        break;
      case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP:
        ROS_WARN_STREAM("Controller " << name_ << " failed with error code OLD_HEADER_TIMESTAMP");
        break;
      case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
        ROS_WARN_STREAM("Controller " << name_ << " failed with error code PATH_TOLERANCE_VIOLATED");
        break;
      case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
        ROS_WARN_STREAM("Controller " << name_ << " failed with error code GOAL_TOLERANCE_VIOLATED");
        break;
    }

    finishControllerExecution(state);
  }

  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM("FollowJointTrajectoryController: " << name_ << " started execution");
  }

  void controllerFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
  {
  }
};


} // end namespace youbot_gripper_moveit_plugin

#endif // MOVEIT_PLUGINS_FOLLOW_TRAJECTORY_CONTROLLER_HANDLE
