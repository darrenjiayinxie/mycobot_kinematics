#include <mycobot_kinematics/mycobot_kinematics_plugin.h>



PLUGINLIB_EXPORT_CLASS(mycobot_kinematics::MycobotKinematicsPlugin, kinematics::KinematicsBase);

namespace mycobot_kinematics
{

	using kinematics::KinematicsResult;
	constexpr char LOGNAME[] = "MYCOBOT";

	MycobotKinematicsPlugin::MycobotKinematicsPlugin() : active_(false){}

	bool MycobotKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
																						const std::string& base_frame, const std::vector<std::string>& tip_frames,
																						double search_discretization)
	{
		bool debug = false;

		ROS_INFO_STREAM_NAMED(LOGNAME, " MycobotKinematicsPlugin initializing");

		storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

		joint_model_group_ = robot_model_->getJointModelGroup(group_name);
		if (!joint_model_group_)
			return false;

		if (debug)
		{
			std::cout << std::endl
								<< "Joint Model Variable Names: "
									"------------------------------------------- "
								<< std::endl;
			const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
			std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
			std::cout << std::endl;
		}

		// Get the dimension of the planning group
		dimension_ = joint_model_group_->getVariableCount();
		ROS_INFO_STREAM_NAMED(LOGNAME, "Dimension planning group '"
																			<< group_name << "': " << dimension_
																			<< ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
																			<< ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

		// Copy joint names
		for (std::size_t i = 0; i < joint_model_group_->getActiveJointModels().size(); ++i)
		{
			ik_group_info_.joint_names.push_back(joint_model_group_->getActiveJointModelNames()[i]);
		}

    // Get base frame pose
    robot_state::RobotState robot_state(robot_model_);
    base_ = robot_state.getGlobalLinkTransform("body_link");
		if (debug)
		{
			ROS_ERROR_STREAM_NAMED(LOGNAME, "tip links available:");
			std::copy(tip_frames_.begin(), tip_frames_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
		}

		// Make sure all the tip links are in the link_names vector
		for (std::size_t i = 0; i < tip_frames_.size(); ++i)
		{
			if (!joint_model_group_->hasLinkModel(tip_frames_[i]))
			{
				ROS_ERROR_NAMED(LOGNAME, "Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(),
												group_name.c_str());
				return false;
			}
			ik_group_info_.link_names.push_back(tip_frames_[i]);
		}
		

		active_ = true;
		ROS_DEBUG_NAMED(LOGNAME, "Mycobot kinematics solver initialized");
		return true;
	}

bool MycobotKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  if (num_possible_redundant_joints_ < 0)
  {
    ROS_ERROR_NAMED(LOGNAME, "This group cannot have redundant joints");
    return false;
  }
  if (redundant_joints.size() > static_cast<std::size_t>(num_possible_redundant_joints_))
  {
    ROS_ERROR_NAMED(LOGNAME, "This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }

  redundant_joint_indices_ = redundant_joints;

  return true;
}

bool MycobotKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

int MycobotKinematicsPlugin::getJointIndex(const std::string& name) const
{
  for (unsigned int i = 0; i < ik_group_info_.joint_names.size(); i++)
  {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool MycobotKinematicsPlugin::timedOut(const ros::WallTime& start_time, double duration) const
{
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}


bool MycobotKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool MycobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MycobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MycobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MycobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MycobotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const std::vector<double>& consistency_limits,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}



bool MycobotKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                 const std::vector<double>& ik_seed_state, double /*timeout*/,
                                                 const std::vector<double>& /*consistency_limits*/,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& /*options*/) const
{ 

  // Check if active
  if (!active_)
  {
    ROS_ERROR_NAMED(LOGNAME, "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check if seed state correct
  if (ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Mismatched number of pose requests (" << ik_poses.size() << ") to tip frames ("
                                                                           << tip_frames_.size()
                                                                           << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  Eigen::Isometry3d pose;
  
  tf::poseMsgToEigen(ik_poses[0], pose);
  pose = base_.inverse()*pose;
  std::vector<std::vector<double>> solutions;
  
  if (!getAllIK(pose, solutions))
  { 
    std::cout << "Failed to find solution" << std::endl;
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Failed to find IK solution");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Now have " << solutions.size() << " potential solutions");

  std::vector<SolDist> Dist_solutions;

  for (auto& sol : solutions)
  {
    Dist_solutions.push_back({ sol, distance(sol, ik_seed_state) });
  }

  // sort solutions by distance to seed state
  std::sort(Dist_solutions.begin(), Dist_solutions.end());
 
  if (!solution_callback)
  {
    solution = Dist_solutions.front().value;
    return true;
  }

  for (auto sol : Dist_solutions)
  {
    solution_callback(ik_poses[0], sol.value, error_code);
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      solution = sol.value;
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Solution passes callback");
      return true;
    }
  }
  
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "No solution fullfilled requirements of solution callback");
  return false;
}

bool MycobotKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                              const std::vector<double>& /*ik_seed_state*/,
                                              std::vector<std::vector<double>>& solutions, KinematicsResult& /*result*/,
                                              const kinematics::KinematicsQueryOptions& /*options*/) const
{
  if (ik_poses.size() > 1 || ik_poses.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "You can only get all solutions for a single pose.");
    return false;
  }
  Eigen::Isometry3d pose;
  pose = base_.inverse()*pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  return getAllIK(pose, solutions);
}

bool MycobotKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                              const std::vector<double>& joint_angles,
                                              std::vector<geometry_msgs::Pose>& poses) const
{
  if (!active_)
  {
    ROS_ERROR_NAMED(LOGNAME, "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  
  Eigen::Isometry3d T;
  forward(joint_angles, T);
  tf::poseEigenToMsg(T, poses[0]);

  return true;
}

const std::vector<std::string>& MycobotKinematicsPlugin::getJointNames() const
{
  return ik_group_info_.joint_names;
}

const std::vector<std::string>& MycobotKinematicsPlugin::getLinkNames() const
{
  return ik_group_info_.link_names;
}

const std::vector<std::string>& MycobotKinematicsPlugin::getVariableNames() const
{
  return joint_model_group_->getVariableNames();
}

double MycobotKinematicsPlugin::distance(const std::vector<double>& a, const std::vector<double>& b)
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::fabs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
std::size_t MycobotKinematicsPlugin::closestJointPose(const std::vector<double>& target,
                                                        const std::vector<std::vector<double>>& candidates)
{
  size_t closest = 0;  // index into candidates
  double lowest_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    assert(target.size() == candidates[i].size());
    double c = distance(target, candidates[i]);
    if (c < lowest_cost)
    {
      closest = i;
      lowest_cost = c;
    }
  }
  return closest;
}

bool MycobotKinematicsPlugin::getAllIK(const Eigen::Isometry3d& pose,
                                         std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();

  // Transform input pose
  // needed if we introduce a tip frame different from tool0
  // or a different base frame
  // Eigen::Isometry3d tool_pose = diff_base.inverse() * pose *
  // tip_frame.inverse();
  
  
  inverse(pose, joint_poses);
  
  return joint_poses.size() > 0;
}

bool MycobotKinematicsPlugin::getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state,
                                      std::vector<double>& joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

}  // namespace mycobot_kinematics_plugin
