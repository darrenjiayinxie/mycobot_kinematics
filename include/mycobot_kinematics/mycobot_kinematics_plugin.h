/* Author: Jiayin Xie (darrenjiayinxie@gmail.com) */

/* Based on orignal source from Willow Garage. License copied below */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#ifndef MYCOBOT_KINEMATICS_PLUGIN_
#define MYCOBOT_KINEMATICS_PLUGIN_

// moveit library
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <tf/transform_datatypes.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// mycobot_kinematics
#include <mycobot_kinematics/mycobot_kin.h>

// time
#include <chrono>

namespace mycobot_kinematics {
	using kinematics::KinematicsResult;
	class MycobotKinematicsPlugin : public kinematics::KinematicsBase {
		public:
      // struct for storing and sorting solutions
      struct SolDist
      {
        std::vector<double> value;
        double dist_from_seed;

        bool operator<(const SolDist& a) const
        {
          return dist_from_seed < a.dist_from_seed;
        }
      };

			MycobotKinematicsPlugin();
			////////////////////////////////overwritten methods//////////////////////////////////////////////////////////
			
			virtual bool
			getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
										std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
										const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			virtual bool
			searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
												std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
												const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;
			
			virtual bool
			searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
												const std::vector<double>& consistency_limits, std::vector<double>& solution,
												moveit_msgs::MoveItErrorCodes& error_code,
												const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			virtual bool
			searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
												std::vector<double>& solution, const IKCallbackFn& solution_callback, 
												moveit_msgs::MoveItErrorCodes& error_code,
												const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			virtual bool
			searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
												const std::vector<double>& consistency_limits, std::vector<double>& solution,
												const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
												const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

			virtual bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
														std::vector<geometry_msgs::Pose>& poses) const override;

			virtual bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
															const std::string& base_frame, const std::vector<std::string>& tip_frames,
															double search_discretization) override;

			/**
			 * @brief  Return all the joint names in the order they are used internally
			 */
			const std::vector<std::string>& getJointNames() const override;

			/**
			 * @brief  Return all the link names in the order they are represented internally
			 */
			const std::vector<std::string>& getLinkNames() const override;

			/**
			 * @brief  Return all the variable names in the order they are represented internally
			 */
			const std::vector<std::string>& getVariableNames() const;

			virtual bool
			getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
											std::vector<std::vector<double>>& solutions, KinematicsResult& result,
											const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

		protected:
			virtual bool
			searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
											std::vector<double>& solution, const IKCallbackFn& solution_callback,
											moveit_msgs::MoveItErrorCodes& error_code, const std::vector<double>& consistency_limits,
											const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

			virtual bool
			searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
											double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
											const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
											const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

			virtual bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices) override;

		private:
			bool timedOut(const ros::WallTime& start_time, double duration) const;

			int getJointIndex(const std::string& name) const;

			bool isRedundantJoint(unsigned int index) const;

			static double distance(const std::vector<double>& a, const std::vector<double>& b);

			static std::size_t closestJointPose(const std::vector<double>& target, const std::vector<std::vector<double>>& candidates);

			bool getAllIK(const Eigen::Isometry3d& pose, std::vector<std::vector<double>>& joint_poses) const;

			bool getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state, std::vector<double>& joint_pose) const;

			bool active_; /** Internal variable that indicates whether solvers are configured and ready */

			moveit_msgs::KinematicSolverInfo ik_group_info_; /** Stores information for the inverse kinematics solver */

			unsigned int dimension_; /** Dimension of the group */

			const robot_model::JointModelGroup* joint_model_group_;

			int num_possible_redundant_joints_;

			Eigen::Isometry3d base_;

	};   
        
} // namespace mycobot_kinematics


#endif // _MYCOBOT_KINEMATICS_PLUGIN_H_