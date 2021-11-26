
#pragma once

#include <array>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>

#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>    //changed from Eigen/Core to eigen3/Eigen/Core to solve a compilation error

#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/control_types.h>


/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 *
 * @return true if successful.
 */
void setDefaultBehavior(franka::Robot* robot);


/**
 * Prints the T_O_EE transformation matrix.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void printCartesianPose(franka::Robot* robot);


/**
 * Prints the 7 joint values of the robot.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void printJointsPosition(franka::Robot* robot);


/**
 * Returns the T_O_EE transformation matrix.
 *
 * @param[in] robot Robot instance to set behavior on.
 *
 * @return an array containing T_O_EE.
 */
std::array<double, 16> getCartesianPose(franka::Robot* robot);


/**
 * Prints the 7 joint values of the robot.
 *
 * @param[in] robot Robot instance to set behavior on.
 *
 * @return an array containing the joints positions.
 */
std::array<double, 7> getJointsPosition(franka::Robot* robot);


/**
 * Moves the robot to the desired cartesian position (no orientation).
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void go2Position(franka::Robot* robot, float x_des, float y_des, float z_des, float motion_duration);


/**
 * Moves the robot to the desired cartesian pose (position + orientation).
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void go2Pose(franka::Robot* robot, float x_des, float y_des, float z_des, double ex, double ey, double ez, double theta, float motion_duration);


/**
 * Brings the robot back to the home position with a user defined speed ratio (from 0 to 1).
 * Uses the default MotionGenerator implemented by libfranka, controlling directly the joints
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void go2JointPositions(franka::Robot* robot, std::array<double, 7> q_goal, double speed_ratio);


/**
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class MotionGenerator {

    public:

        /**
        * Creates a new MotionGenerator instance for a target q.
        *
        * @param[in] speed_factor General speed factor in range [0, 1].
        * @param[in] q_goal Target joint positions.
        */
        MotionGenerator(double speed_factor, const std::array<double, 7> q_goal);

        /**
        * Sends joint position calculations
        *
        * @param[in] robot_state Current state of the robot.
        * @param[in] period Duration of execution.
        *
        * @return Joint positions for use inside a control loop.
        */
        franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);


    private:

        using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
        using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

        bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;

        void calculateSynchronizedValues();

        static constexpr double kDeltaQMotionFinished = 1e-6;
        const Vector7d q_goal_;

        Vector7d q_start_;
        Vector7d delta_q_;

        Vector7d dq_max_sync_;
        Vector7d t_1_sync_;
        Vector7d t_2_sync_;
        Vector7d t_f_sync_;
        Vector7d q_1_;

        double time_ = 0.0;

        Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
        Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
        Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();

};
