#ifndef MOVEIT_PLUGINS_GRIPPER_CONTROLLER_HANDLE
#define MOVEIT_PLUGINS_GRIPPER_CONTROLLER_HANDLE

#include <youbot_gripper_moveit_plugin/action_based_controller_handle.h>
#include <control_msgs/GripperCommandAction.h>
#include <set>

namespace youbot_gripper_moveit_plugin
{

/*
 * This is an interface for a gripper using control_msgs/GripperCommandAction
 * action interface (single DOF).
 */
class GripperControllerHandle :
      public ActionBasedControllerHandle<control_msgs::GripperCommandAction>
{
public:
  /* Topics will map to name/ns/goal, name/ns/result, etc */
  GripperControllerHandle(const std::string &name, const std::string &ns) :
    ActionBasedControllerHandle<control_msgs::GripperCommandAction>(name, ns),
    allow_failure_(false), parallel_jaw_gripper_(false)
  {
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
  {
    ROS_DEBUG_STREAM_NAMED("GripperController",
                           "Received new trajectory for " << name_);

    if (!controller_action_client_)
      return false;

    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR_NAMED("GripperController",
                      "Gripper cannot execute multi-dof trajectories.");
      return false;
    }

    if (trajectory.joint_trajectory.points.empty())
    {
      ROS_ERROR_NAMED("GripperController",
                      "GripperController requires at least one joint trajectory point.");
      return false;
    }

    if (trajectory.joint_trajectory.points.size() > 1)
    {
      ROS_DEBUG_STREAM_NAMED("GripperController","Trajectory: " << trajectory.joint_trajectory);
    }

    if (trajectory.joint_trajectory.joint_names.empty())
    {
      ROS_ERROR_NAMED("GripperController", "No joint names specified");
      return false;
    }

    std::vector<int> gripper_joint_indexes;
    for (std::size_t i = 0 ; i < trajectory.joint_trajectory.joint_names.size() ; ++i)
    {
      if (command_joints_.find(trajectory.joint_trajectory.joint_names[i]) != command_joints_.end())
      {
        gripper_joint_indexes.push_back(i);
        if (!parallel_jaw_gripper_)
          break;
      }
    }

    if (gripper_joint_indexes.empty())
    {
      ROS_WARN_NAMED("GripperController",
                     "No command_joint was specified for the MoveIt controller gripper handle. \
                      Please see GripperControllerHandle::addCommandJoint() and \
                      GripperControllerHandle::setCommandJoint(). Assuming index 0.");
      gripper_joint_indexes.push_back(0);
    }

    // goal to be sent
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 0.0;

    // send last point
    int tpoint = trajectory.joint_trajectory.points.size() - 1;
    ROS_DEBUG_NAMED("GripperController",
                    "Sending command from trajectory point %d",
                    tpoint);

    // fill in goal from last point
    for (std::size_t i = 0; i < gripper_joint_indexes.size(); ++i)
    {
      int idx = gripper_joint_indexes[i];

      if (trajectory.joint_trajectory.points[tpoint].positions.size() <= idx)
      {
        ROS_ERROR_NAMED("GripperController",
                        "GripperController expects a joint trajectory with one \
                         point that specifies at least the position of joint \
                         '%s', but insufficient positions provided",
                        trajectory.joint_trajectory.joint_names[idx].c_str());
        return false;
      }
      goal.command.position += trajectory.joint_trajectory.points[tpoint].positions[idx];

      if (trajectory.joint_trajectory.points[tpoint].effort.size() > idx)
        goal.command.max_effort = trajectory.joint_trajectory.points[tpoint].effort[idx];
    }

    controller_action_client_->sendGoal(goal,
                    boost::bind(&GripperControllerHandle::controllerDoneCallback, this, _1, _2),
                    boost::bind(&GripperControllerHandle::controllerActiveCallback, this),
                    boost::bind(&GripperControllerHandle::controllerFeedbackCallback, this, _1));

    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  void setCommandJoint(const std::string& name)
  {
    command_joints_.clear();
    addCommandJoint(name);
  }

  void addCommandJoint(const std::string& name)
  {
    command_joints_.insert(name);
  }

  void allowFailure(bool allow)
  {
    allow_failure_ = allow;
  }

  void setParallelJawGripper(const std::string& left, const std::string& right)
  {
    addCommandJoint(left);
    addCommandJoint(right);
    parallel_jaw_gripper_ = true;
  }

private:

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::GripperCommandResultConstPtr& result)
  {
    if (state == actionlib::SimpleClientGoalState::ABORTED && allow_failure_)
      finishControllerExecution(actionlib::SimpleClientGoalState::SUCCEEDED);
    else
      finishControllerExecution(state);
  }

  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM_NAMED("GripperController", name_ << " started execution");
  }

  void controllerFeedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr& feedback)
  {
  }

  /*
   * Some gripper drivers may indicate a failure if they do not close all the way when
   * an object is in the gripper.
   */
  bool allow_failure_;

  /*
   * A common setup is where there are two joints that each move
   * half the overall distance. Thus, the command to the gripper
   * should be the sum of the two joint distances.
   *
   * When this is set, command_joints_ should be of size 2,
   * and the command will be the sum of the two joints.
   */
  bool parallel_jaw_gripper_;

  /*
   * A GripperCommand message has only a single float64 for the
   * "command", thus only a single joint angle can be sent -- however,
   * due to the complexity of making grippers look correct in a URDF,
   * they typically have >1 joints. The "command_joint" is the joint
   * whose position value will be sent in the GripperCommand action. A
   * set of names is provided for acceptable joint names. If any of
   * the joints specified is found, the value corresponding to that
   * joint is considered the command.
   */
  std::set<std::string> command_joints_;
};


} // end namespace youbot_gripper_moveit_plugin

#endif // MOVEIT_PLUGINS_GRIPPER_CONTROLLER_HANDLE
