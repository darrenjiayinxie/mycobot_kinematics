
/* 
 * Provides forward and inverse kinematics for Mycobot robot
 * Author: Jiayin Xie (darrenjiayinxie@gmail.com)
*/

#ifndef MYCOBOT_KIN_H
#define MYCOBOT_KIN_H

// library includes
#include <vector>
#include <cmath>
#include <iostream>
#include <random>
#include <ctime>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


namespace mycobot_kinematics {
    

    /**
     * @brief    Given a set of 6 joint angles, compute the end effector pose
     * @param q  The 6 joint values of mycobot
     * @param T  The 4x4 end effector pose 
     * @return   void
     */
    void forward(const std::vector<double>& q, Eigen::Isometry3d& T);

    /**
     * @brief         Wrap angle in radians to [0 2*PI)
     * @param angle   angle in radians
     * @return        void
     */
    void wrapTo2Pi(double& angle);

    /**
     * @brief         Wrap angle in radians to [−PI PI)
     * @param angle   angle in radians
     * @return        void
     */
    void wrapToPi(double& angle);

    

    /**
     * @brief            Given the one candidate of angles computed in previous steps, compute the angle in current steps
     * @param T          The 4x4 end effector pose 
     * @param candidate  The set of candidate solution from previous steps
     * @param step       The index of current step (0-indexed)
     * @return           The angle(s) in current step depend on given angles
     */
    std::vector<double> cal_angle(const Eigen::Isometry3d& T, const std::vector<double>& candidate, const int step);

    /**
     * @brief            The preorder tree traversal algorithm to validate and find the possible solutions 
     * @param T          The 4x4 end effector pose 
     * @param step       The index of current step (0-indexed)
     * @param candidate  The set of candidate solution from previous steps
     * @param solutions  The set of possible solutions
     * @return           void
     */
    void solutions_tree(const Eigen::Isometry3d& T, int step, std::vector<double>& candidate, std::vector<std::vector<double>>& solutions);


    /**
     * @brief         Given the 4x4 end effector pose, compute the set of 6 joint angles
     * @param q_sols  A matrix of doubles returned, all angles should be in [-PI,PI]
     * @return        void
     */
    void inverse(const Eigen::Isometry3d& T, std::vector<std::vector<double>>& q_sols);

    
} // namespace mycobot_kinematics




#endif //MYCOBOT_KIN_H