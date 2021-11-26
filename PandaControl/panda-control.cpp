
#include "panda-control.h"


void setDefaultBehavior(franka::Robot* robot) {
    try{
//        robot->setCollisionBehavior(
//            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
//            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot->setCollisionBehavior(
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
//        return -1;
    }
}


void printCartesianPose(franka::Robot* robot) {
    robot->read([](const franka::RobotState& robot_state) {
        std::array<double, 16> current_pose = robot_state.O_T_EE_c;
        std::streamsize cp = std::cout.precision();
        std::cout.precision(3);
        std::cout << std::endl;
        std::cout << "T_ee_base  =  [  " << current_pose[0] << "          "  << current_pose[4] << "          "  << current_pose[8]  << "          "  << current_pose[12] << "  ]" << std::endl;
        std::cout << "              [  " << current_pose[1] << "          "  << current_pose[5] << "          "  << current_pose[9]  << "          "  << current_pose[13] << "  ]" << std::endl;
        std::cout << "              [  " << current_pose[2] << "          "  << current_pose[6] << "          "  << current_pose[10] << "          "  << current_pose[14] << "  ]" << std::endl;
        std::cout << "              [  " << current_pose[3] << "          "  << current_pose[7] << "          "  << current_pose[11] << "          "  << current_pose[15] << "  ]" << std::endl;
        std::cout.precision(cp);
        return false;
    });
}


void printJointsPosition(franka::Robot* robot) {
    robot->read([](const franka::RobotState& robot_state) {
        std::array<double, 7> current_joints = robot_state.q;
        std::cout << std::endl;
        std::cout << "q  =  [ " << current_joints[0] << "   "  << current_joints[1] << "   "  << current_joints[2]  << "   "
                                << current_joints[3] << "   "  << current_joints[4] << "   "  << current_joints[5]  << "   "
                                << current_joints[6] << "  ]" << std::endl;
        return false;
    });
}


std::array<double, 16> getCartesianPose(franka::Robot* robot) {
    std::array<double, 16> current_pose;
    robot->read([&current_pose](const franka::RobotState& robot_state) {
        current_pose = robot_state.O_T_EE_c;
        return false;
    });
    return current_pose;
}


std::array<double, 7> getJointsPosition(franka::Robot* robot) {
    std::array<double, 7> current_joints;
    robot->read([&current_joints](const franka::RobotState& robot_state) {
        current_joints = robot_state.q;
        return false;
    });
    return current_joints;
}


void go2Position(franka::Robot* robot, float x_des, float y_des, float z_des, float motion_duration) {

    double x_i;   double y_i;   double z_i;
    double xd_i;  double yd_i;  double zd_i;
    double xdd_i; double ydd_i; double zdd_i;
    double x_f;   double y_f;   double z_f;
    double xd_f;  double yd_f;  double zd_f;
    double xdd_f; double ydd_f; double zdd_f;

    try {
        double time = 0.0;
        std::array<double, 16> initial_pose, new_pose;

        robot->control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
            time += period.toSec();

            if (time == 0.0) {
                initial_pose = robot_state.O_T_EE_c;
                new_pose = initial_pose;

                //TODO: vectorize everything and pass velocity and acceleration
                x_i = initial_pose[12]; y_i = initial_pose[13]; z_i = initial_pose[14];
                xd_i = 0;               yd_i = 0;               zd_i = 0;
                xdd_i = 0;              ydd_i = 0;              zdd_i = 0;
                x_f = x_des;            y_f = y_des;            z_f = z_des;
                xd_f = 0;               yd_f = 0;               zd_f = 0;
                xdd_f = 0;              ydd_f = 0;              zdd_f = 0;
            }

            /* 3rd order polynomia to generate suitable position, velocity and acceleration references */
            /*
            double a0x = initial_pose[12];  double a0y = initial_pose[13];  double a0z = initial_pose[14];
            double a1x = 0;                 double a1y = 0;                 double a1z = 0;
            double a3x = -2*(xd-initial_pose[12]-motion_duration/2*(a1x+0))/pow(motion_duration, 3);
            double a3y = -2*(yd-initial_pose[13]-motion_duration/2*(a1y+0))/pow(motion_duration, 3);
            double a3z = -2*(zd-initial_pose[14]-motion_duration/2*(a1z+0))/pow(motion_duration, 3);
            double a2x = (-a1x-3*a3x*pow(motion_duration, 2))/(2*motion_duration);
            double a2y = (-a1y-3*a3y*pow(motion_duration, 2))/(2*motion_duration);
            double a2z = (-a1z-3*a3z*pow(motion_duration, 2))/(2*motion_duration);

            new_pose[12] = a3x*pow(time, 3) + a2x*pow(time, 2) + a1x*time + a0x;
            new_pose[13] = a3y*pow(time, 3) + a2y*pow(time, 2) + a1y*time + a0y;
            new_pose[14] = a3z*pow(time, 3) + a2z*pow(time, 2) + a1z*time + a0z;
            */

            /* 5th order polynomia to generate suitable position, velocity and acceleration references */
            double a0x = x_i;       double a0y = y_i;       double a0z = z_i;
            double a1x = xd_i;      double a1y = yd_i;      double a1z = zd_i;
            double a2x = 0.5*xdd_i; double a2y = 0.5*ydd_i; double a2z = 0.5*zdd_i;
            double a3x = (20*(x_f-x_i)-(8*xd_f+12*xd_i)*motion_duration-(3*xdd_f-xdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,3));
            double a3y = (20*(y_f-y_i)-(8*yd_f+12*yd_i)*motion_duration-(3*ydd_f-ydd_i)*pow(motion_duration,2))/(2*pow(motion_duration,3));
            double a3z = (20*(z_f-z_i)-(8*zd_f+12*zd_i)*motion_duration-(3*zdd_f-zdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,3));
            double a4x = (30*(x_i-x_f)+(14*xd_f+16*xd_i)*motion_duration+(3*xdd_f-2*xdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,4));
            double a4y = (30*(y_i-y_f)+(14*yd_f+16*yd_i)*motion_duration+(3*ydd_f-2*ydd_i)*pow(motion_duration,2))/(2*pow(motion_duration,4));
            double a4z = (30*(z_i-z_f)+(14*zd_f+16*zd_i)*motion_duration+(3*zdd_f-2*zdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,4));
            double a5x = (12*(x_f-x_i)-(6*xd_f+xd_i)*motion_duration-(xdd_f-xdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,5));
            double a5y = (12*(y_f-y_i)-(6*yd_f+yd_i)*motion_duration-(ydd_f-ydd_i)*pow(motion_duration,2))/(2*pow(motion_duration,5));
            double a5z = (12*(z_f-z_i)-(6*zd_f+zd_i)*motion_duration-(zdd_f-zdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,5));

            new_pose[12] = a5x*pow(time,5) + a4x*pow(time,4) + a3x*pow(time,3) + a2x*pow(time,2) + a1x*time + a0x;
            new_pose[13] = a5y*pow(time,5) + a4y*pow(time,4) + a3y*pow(time,3) + a2y*pow(time,2) + a1y*time + a0y;
            new_pose[14] = a5z*pow(time,5) + a4z*pow(time,4) + a3z*pow(time,3) + a2z*pow(time,2) + a1z*time + a0z;

            if (time >= (motion_duration)) {
//                std::cout << std::endl << "Finished motion" << std::endl;
                return franka::MotionFinished(new_pose);
            }
            return new_pose;
        });
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        // return -1;
    }
}


void go2Pose(franka::Robot* robot, float x_des, float y_des, float z_des, double ex, double ey, double ez, double theta, float motion_duration) {

    double x_i;   double y_i;   double z_i;
    double xd_i;  double yd_i;  double zd_i;
    double xdd_i; double ydd_i; double zdd_i;
    double x_f;   double y_f;   double z_f;
    double xd_f;  double yd_f;  double zd_f;
    double xdd_f; double ydd_f; double zdd_f;

    Eigen::AngleAxisd aa_des(theta, Eigen::Vector3d (ex, ey, ez));
    Eigen::Matrix3d init_o;
    Eigen::Matrix3d des_o(aa_des);

    try {
        double time = 0.0;
        std::array<double, 16> initial_pose, new_pose;

        robot->control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
            time += period.toSec();

            if (time == 0.0) {
                initial_pose = robot_state.O_T_EE_c;
                new_pose = initial_pose;

                //TODO: vectorize everything and pass velocity and acceleration
                x_i = initial_pose[12]; y_i = initial_pose[13]; z_i = initial_pose[14];
                xd_i = 0;               yd_i = 0;               zd_i = 0;
                xdd_i = 0;              ydd_i = 0;              zdd_i = 0;
                x_f = x_des;            y_f = y_des;            z_f = z_des;
                xd_f = 0;               yd_f = 0;               zd_f = 0;
                xdd_f = 0;              ydd_f = 0;              zdd_f = 0;

                //rotMatrix da initial_pose
                init_o(0) = initial_pose[0]; init_o(1) = initial_pose[4]; init_o(2)  = initial_pose[8];
                init_o(3) = initial_pose[1]; init_o(4) = initial_pose[5]; init_o(5)  = initial_pose[9];
                init_o(6) = initial_pose[2]; init_o(7) = initial_pose[6]; init_o(8) = initial_pose[10];

//                std::cout << std::endl << des_o << std::endl;

            }

            /* 5th order polynomia to generate suitable position, velocity and acceleration references */
            double a0x = x_i;       double a0y = y_i;       double a0z = z_i;
            double a1x = xd_i;      double a1y = yd_i;      double a1z = zd_i;
            double a2x = 0.5*xdd_i; double a2y = 0.5*ydd_i; double a2z = 0.5*zdd_i;
            double a3x = (20*(x_f-x_i)-(8*xd_f+12*xd_i)*motion_duration-(3*xdd_f-xdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,3));
            double a3y = (20*(y_f-y_i)-(8*yd_f+12*yd_i)*motion_duration-(3*ydd_f-ydd_i)*pow(motion_duration,2))/(2*pow(motion_duration,3));
            double a3z = (20*(z_f-z_i)-(8*zd_f+12*zd_i)*motion_duration-(3*zdd_f-zdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,3));
            double a4x = (30*(x_i-x_f)+(14*xd_f+16*xd_i)*motion_duration+(3*xdd_f-2*xdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,4));
            double a4y = (30*(y_i-y_f)+(14*yd_f+16*yd_i)*motion_duration+(3*ydd_f-2*ydd_i)*pow(motion_duration,2))/(2*pow(motion_duration,4));
            double a4z = (30*(z_i-z_f)+(14*zd_f+16*zd_i)*motion_duration+(3*zdd_f-2*zdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,4));
            double a5x = (12*(x_f-x_i)-(6*xd_f+xd_i)*motion_duration-(xdd_f-xdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,5));
            double a5y = (12*(y_f-y_i)-(6*yd_f+yd_i)*motion_duration-(ydd_f-ydd_i)*pow(motion_duration,2))/(2*pow(motion_duration,5));
            double a5z = (12*(z_f-z_i)-(6*zd_f+zd_i)*motion_duration-(zdd_f-zdd_i)*pow(motion_duration,2))/(2*pow(motion_duration,5));


            double a0s = 0;
            double a1s = 0;
            double a2s = 0;
            double a3s = 20/(2*pow(motion_duration,3));
            double a4s = -30/(2*pow(motion_duration,4));
            double a5s = 12/(2*pow(motion_duration,5));
            double s = a5s*pow(time,5) + a4s*pow(time,4) + a3s*pow(time,3) + a2s*pow(time,2) + a1s*time + a0s;;
            Eigen::Matrix3d curr_o = init_o*((init_o.transpose()*des_o).log()*s).exp();

//            if (time==0.0) {
//                std::cout << std::endl << curr_o << std::endl;
//                std::cout << std::endl << curr_o.isUnitary() << std::endl;
//            }

            //commmand new orientation
            new_pose[0] = curr_o(0); new_pose[4] = curr_o(1); new_pose[8]  = curr_o(2);
            new_pose[1] = curr_o(3); new_pose[5] = curr_o(4); new_pose[9]  = curr_o(5);
            new_pose[2] = curr_o(6); new_pose[6] = curr_o(7); new_pose[10] = curr_o(8);

            new_pose[12] = a5x*pow(time,5) + a4x*pow(time,4) + a3x*pow(time,3) + a2x*pow(time,2) + a1x*time + a0x;
            new_pose[13] = a5y*pow(time,5) + a4y*pow(time,4) + a3y*pow(time,3) + a2y*pow(time,2) + a1y*time + a0y;
            new_pose[14] = a5z*pow(time,5) + a4z*pow(time,4) + a3z*pow(time,3) + a2z*pow(time,2) + a1z*time + a0z;

            if (time >= (motion_duration)) {
//                std::cout << std::endl << "Finished motion" << std::endl;
                return franka::MotionFinished(new_pose);
            }
            return new_pose;
        });
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        // return -1;
    }
}


void go2JointPositions(franka::Robot* robot, std::array<double, 7> q_goal, double speed_ratio=0.1) {
    try {
        setDefaultBehavior(robot);

        MotionGenerator motion_generator(speed_ratio, q_goal);
//        std::cout << std::endl << "WARNING: This example will move the robot in 2 seconds! "
//                  << "Please move away and make sure to have the user stop button at hand!" << std::endl;
//                  << "Press Enter to continue..." << std::endl;
//        std::cin.ignore();
//        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        robot->control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        // return -1;
    }
}


MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal) : q_goal_(q_goal.data()) {
    dq_max_ *= speed_factor;
    ddq_max_start_ *= speed_factor;
    ddq_max_goal_ *= speed_factor;
    dq_max_sync_.setZero();
    q_start_.setZero();
    delta_q_.setZero();
    t_1_sync_.setZero();
    t_2_sync_.setZero();
    t_f_sync_.setZero();
    q_1_.setZero();
}


bool MotionGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();
    Vector7d t_d = t_2_sync_ - t_1_sync_;
    Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
    std::array<bool, 7> joint_motion_finished{};

    for (size_t i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
            (*delta_q_d)[i] = 0;
            joint_motion_finished[i] = true;
        } else {
            if (t < t_1_sync_[i]) {
                (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] *
                        sign_delta_q[i] * (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
            } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) { (*delta_q_d)[i] = q_1_[i] +
                        (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
            } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) { (*delta_q_d)[i] = delta_q_[i] +
                        0.5 * (1.0 / std::pow(delta_t_2_sync[i], 3.0) * (t - t_1_sync_[i] - 2.0 *
                        delta_t_2_sync[i] - t_d[i]) * std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                        (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) * dq_max_sync_[i] * sign_delta_q[i];
            } else {
                (*delta_q_d)[i] = delta_q_[i];
                joint_motion_finished[i] = true;
            }
        }
    }
    return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(), [](bool x) { return x; });
}


void MotionGenerator::calculateSynchronizedValues() {
    Vector7d dq_max_reach(dq_max_);
    Vector7d t_f = Vector7d::Zero();
    Vector7d delta_t_2 = Vector7d::Zero();
    Vector7d t_1 = Vector7d::Zero();
    Vector7d delta_t_2_sync = Vector7d::Zero();
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();

    for (size_t i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
            if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) + 3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
                dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] * (ddq_max_start_[i] * ddq_max_goal_[i]) / (ddq_max_start_[i] + ddq_max_goal_[i]));
            }
            t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
            delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
            t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
        }
    }

    double max_t_f = t_f.maxCoeff();
    for (size_t i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
            double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
            double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
            double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
            double delta = b * b - 4.0 * a * c;
            if (delta < 0.0) { delta = 0.0; }
            dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
            t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
            delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
            t_f_sync_[i] = (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
            t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
            q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
        }
    }
}


franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state, franka::Duration period) {
    time_ += period.toSec();

    if (time_ == 0.0) {
        q_start_ = Vector7d(robot_state.q_d.data());
        delta_q_ = q_goal_ - q_start_;
        calculateSynchronizedValues();
    }

    Vector7d delta_q_d;
    bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

    std::array<double, 7> joint_positions;
    Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
    franka::JointPositions output(joint_positions);
    output.motion_finished = motion_finished;
    return output;
}
