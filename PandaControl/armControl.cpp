
#include <yarp/os/all.h>
#include <event-driven/all.h>
#include <sciplot/sciplot.hpp>
#include <boost/circular_buffer.hpp>
#include <numeric>
#include "panda-control.h"

using namespace yarp::os;
using namespace ev;
using namespace sciplot;


//// ORIGINAL SET OF PARAMETERS
//// set robot parameters for collision behavior
//void setRobotCollisionBehavior(franka::Robot* robot) {
//    robot->setCollisionBehavior(
//        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
//    robot->setJointImpedance({{1000, 1000, 1000, 50, 50, 10, 10}});
//    robot->setCartesianImpedance({{1000, 1000, 1000, 10, 10, 10}});
//}


// set robot parameters for collision behavior
void setRobotCollisionBehavior(franka::Robot* robot) {
    robot->setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot->setJointImpedance({{3000, 3000, 3000, 3000, 500, 500, 500}});
    robot->setCartesianImpedance({{2000, 2000, 2000, 100, 100, 100}});
}

float pxl2cm(double y_coord) {
    return 7.13504e-07*pow(y_coord, 2) - 0.00495117*y_coord + 0.89539;
}

class PandaCartesianController : public RFModule, public Thread {

    private:

        float u_target, v_target, eta;  // predicted (u,v) spatial target and time left
        float prec_target;              // last height target [cm]
        float time_threshold;           // time left to wait before moving
        float period;                   // updateModule period
        float max_robot_height, min_robot_height, init_robot_height, time_between_limits;

        bool can_move;                  // can the robot move?

        std::array<double, 7> q_goal;   // joint values

        std::string robot_ip;           // robot ip address
        franka::Robot *robot;           // robot object instance

        RpcServer handler_port;         // rpc port
        BufferedPort<Bottle> prediction;    // prediction input port

        // Convergence analysis varsg
        int convergence_numPts;         // number of points to consider for evaluating convergence
        bool is_converged;              // has the prediction converged?
        float convergence_thresh;       // maximum displacement in cm allowed in convergence_numPts
        float robot_velocity;           // robot velocity between the two motion extremes
        boost::circular_buffer<float> convergence_pts;  // container for the latest numPts prediction points to evaluate convergence

        // Visualization vars
        Plot plot_t, plot_y, plot_convergence;                      // sciplot plot object
        std::mutex image_mutex;         // mutex for managing points to plot
        std::vector<double> time_intervals, time_prediction, y_tracker, y_predictor, convergence_series;     // vectors containing dt, y_tracker, and y_prediction for plotting

    public:

    PandaCartesianController() { robot = nullptr; }

    virtual bool configure(yarp::os::ResourceFinder& rf) {
        //set the module name used to name ports
        setName((rf.check("name", Value("/panda_controller")).asString()).c_str());
        //set synchronous module period
        period = rf.check("period", Value(0.001)).asFloat64();
        //set time threshold for motion to begin
        time_threshold = rf.check("time_thresh", Value(0.65)).asFloat64();
        //open io ports
        if(!prediction.open(getName() + "/prediction:i")) {
            yError() << "Could not open prediction input port";
            return false;
        }
        //open rpc port
        handler_port.open(getName()+"/rpc:i");
        attach(handler_port);
        //read flags and parameters
        robot_ip = rf.check("robot_ip", Value("172.16.0.2")).asString();

        // automatic connection (bad practice)
        yarp::os::Network::connect("/vPredictor/tracker_prediction:o", "/panda_controller/prediction:i");

        // experiment and robot specific params
        max_robot_height  = 0.86;               // maximum height allowed [cm]
        min_robot_height  = 0.26;               // minimum height allowed [cm]
        time_between_limits = 1.1;              // minimum time needed to move between max_robot_height and min_robot_height
        init_robot_height = (max_robot_height+min_robot_height)/2;
        robot_velocity = (0.56 - 0.26)/0.7;     //(max_robot_height-min_robot_height)/time_between_limits;
        prec_target = init_robot_height;
        convergence_numPts = 25;                // number of points to consider for the convergence criteria
        convergence_thresh = 0.10;
        convergence_pts.set_capacity(convergence_numPts);

        // go to initial position
        try {
            robot = new franka::Robot(robot_ip);
            q_goal = {{0.058979, -0.852004, -0.0372719, -2.63378, -0.0670979, 1.80709, 0.0526138}};
            go2JointPositions(robot, q_goal, 0.5);
            go2Pose(robot, 0.40, -0.25, init_robot_height, 0.5773503, -0.5773503, 0.5773503, 4.1887902, 3.5);
            setRobotCollisionBehavior(robot);
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
            std::cout << "Running error recovery..." << std::endl;
            robot->automaticErrorRecovery();
            return -1;
        }

        // plot object initialization
        plot_t.xlabel("Time [s]");
        plot_t.ylabel("Time [s]");
        plot_t.legend()               // Set the legend to be on the bottom along the horizontal
            .atOutsideBottom()
            .displayHorizontal()
            .displayExpandWidthBy(2);

        plot_y.xlabel("Time [s]");
        plot_y.ylabel("Y [m]");
        plot_y.legend()               // Set the legend to be on the bottom along the horizontal
            .atOutsideBottom()
            .displayHorizontal()
            .displayExpandWidthBy(2);
        plot_y.yrange(0.0, 1.0);

        plot_convergence.xlabel("Time [s]");
        plot_convergence.ylabel("Gamma");
        plot_convergence.legend()               // Set the legend to be on the bottom along the horizontal
            .atOutsideBottom()
            .displayHorizontal()
            .displayExpandWidthBy(2);

        resetTrial();

        yInfo() << "You can throw the ball!";

        return Thread::start();
    }

    virtual bool updateModule() {

/*
       // control check with continous movements
        while (true) {
            go2Pose(robot, 0.40, -0.25, max_robot_height, 0.5773503, -0.5773503, 0.5773503, 4.1887902, 0.7);
            Time::delay(0.5);
            go2Pose(robot, 0.40, -0.25, init_robot_height, 0.5773503, -0.5773503, 0.5773503, 4.1887902, 0.7);
            Time::delay(0.5);
            go2Pose(robot, 0.40, -0.25, min_robot_height, 0.5773503, -0.5773503, 0.5773503, 4.1887902, 0.7);
            Time::delay(0.5);
            go2Pose(robot, 0.40, -0.25, init_robot_height, 0.5773503, -0.5773503, 0.5773503, 4.1887902, 0.7);
            Time::delay(0.5);
        }
*/


/*
        if (is_converged && v_target >= 30 && v_target <= 145 && u_target <= 100) {
//            double mapped_v_target = 0.17 + (139.0 - (float)v_target)*(((45.0-17.0)/100.0)/(139.0-76.0));   // visual2task space mapping
            float mapped_v_target = pxl2cm(v_target);                               // visual2task space mapping
            float target = std::max(min_robot_height, mapped_v_target);             // take the maximum between the predicted position and the minimum allowed height
            float t_diff = eta - (target-min_robot_height)/robot_velocity;          // difference between how much time is left and how much time the robot would need to move
            std::cout << " t_diff = "  << t_diff << " eta = "  << eta << "s at y = " << pxl2cm(v_target) << "cm   ... ";
            if (t_diff < 0.0) {
                std::cout << " too late :(" << std::endl;
                createImage();
                Time::delay(5);
                go2Position(robot, 0.40, -0.25, min_robot_height, 1.0);
                resetTrial();
            } else if (t_diff < 0.05) {
                go2Position(robot, 0.40, -0.25, target, eta);
                std::cout << " target = " << target << std::endl;
                createImage();
                Time::delay(5);
                go2Position(robot, 0.40, -0.25, min_robot_height, 1.0);
                resetTrial();
            } else {
                std::cout << "You can wait..." << std::endl;
            }
        }
*/


//        // MOVE AT THE LAST AVAILABLE TIME
//        if (is_converged) {

//            float target = std::max(min_robot_height, pxl2cm(v_target));            // take the maximum between the predicted position and the minimum allowed height
//            float t_diff = eta - (target-init_robot_height)/robot_velocity;          // difference between how much time is left and how much time the robot would need to move

//            yInfo() << "eta = " << eta << "  -  time needed = " << (target-init_robot_height)/robot_velocity << "   -  target = " << target;

//            if (t_diff < 0.0) {
//                std::cout << " too late :(" << std::endl;
//                std::cout << "Press Enter to move the robot: ";
//                if (std::cin.get() == '\n'){
////                    createImage();
////                    Time::delay(5);
//                    resetTrial();
//                }
////                createImage();
////                go2Position(robot, 0.40, -0.25, init_robot_height, 1.0);
////                resetTrial();
//            } else if (t_diff < 0.1) {
//                go2Position(robot, 0.40, -0.25, target, eta);
//                std::cout << " target = " << target << std::endl;
//                std::cout << "Press Enter to move the robot: ";
//                if (std::cin.get() == '\n'){
////                    createImage();
////                    Time::delay(5);
//                    resetTrial();
//                }
//                go2Position(robot, 0.40, -0.25, init_robot_height, 1.0);
////                createImage();
//                resetTrial();
//            } else {
////                std::cout << "You can wait..." << std::endl;
//            }


        char flag;

        // MOVE AT THE FIRST CONVERGENCE INSTANT
        if (is_converged) {
            float target = std::max(min_robot_height, pxl2cm(v_target));                                // take the maximum between the predicted position and the minimum allowed height
            go2Position(robot, 0.40, -0.25, target, eta); //(target-init_robot_height)/robot_velocity);         // you can use either eta or the real time needed
//            std::cout << " target = " << target << std::endl;
//            Time::delay(1);
            std::cout << "Enter 'p' to plot the data or any other character to restart the trial: ";
            std::cin >> flag;
            if (flag == 'p') {
                createImage();
                std::cout << "Data plotted!" << std::endl;
//                Time::delay(1);
                std::cout << "Enter any character to reset the trial: ";
                std::cin >> flag;
            }
            go2Position(robot, 0.40, -0.25, init_robot_height, 1.0);
            resetTrial();
            std::cout << "Trial resetted! You can throw the ball again!" << std::endl;
        }

/*
        // if the robot has moved, reset it after 2s
        if (motion_done && Time::now()-t0 > 2.0){
            go2Pose(robot, 0.40, -0.45, 0.45, 0.7193398, 0.6946584, 0.0, 3.14159, 3.0);
            setRobotCollisionBehavior(robot);
            motion_done = false;
            convergence_pts.clear();
            yInfo() << "Robot back to home position...";
        }
*/

        return Thread::isRunning();
    }

    void run() {

        float dt = 0.0, x = 0.0, y = 0.0;
        float tot_variation;

        while(true) {
            Bottle *b = prediction.read();

            dt  = b->get(0).asFloat64();
            x   = b->get(1).asFloat64();
            y   = b->get(2).asFloat64();
            eta = b->get(3).asFloat64();
            u_target  = b->get(4).asFloat64();

            // check convergence criteria (must wait second point to compute first correction)
            if (v_target > 0.0) {
                // push the difference between old v_target and current one, before updating it outside the if
                convergence_pts.push_front(abs(pxl2cm(v_target) - pxl2cm(b->get(5).asFloat64())));
                // if buffer full, start checking convergence
                if (convergence_pts.size() == convergence_numPts) {
                    tot_variation = std::accumulate(convergence_pts.begin(), convergence_pts.end(), 0.0);
                    if (tot_variation < convergence_thresh)
                        is_converged = true;
                }
            }

            v_target = b->get(5).asFloat64();

            // push data for later plotting
            image_mutex.lock();
            convergence_series.push_back(tot_variation);
            time_intervals.push_back(dt);
            time_prediction.push_back(eta);
            y_tracker.push_back(pxl2cm(y));
            y_predictor.push_back(pxl2cm(v_target));
            image_mutex.unlock();
        }
    }

    void createImage() {
        if (time_intervals.size() > 0) {

            image_mutex.lock();

            float t, time_cum_prec = 0.0;
            std::vector<double> time_cum;
            for (auto i=0; i<time_intervals.size(); i++) {
                t = time_cum_prec + time_intervals[i];
                time_cum.push_back(t);
                time_cum_prec = t;
                time_prediction[i] = time_prediction[i] + t;
            }

            std::vector<double> t_ground_truth(time_intervals.size(), time_cum.back());
            std::vector<double> y_ground_truth(time_intervals.size(), y_tracker.back());
            std::vector<double> gamma_threshold(time_intervals.size(), convergence_thresh);

            plot_t.drawCurve(time_cum, t_ground_truth).label("final value");
            plot_t.drawCurve(time_cum, time_cum).label("tracker");
            plot_t.drawCurve(time_cum, time_prediction).label("prediction");
            plot_t.xrange(0.0, time_cum.back());
            plot_t.yrange(0.0, 1.5);

            plot_y.drawCurve(time_cum, y_ground_truth).label("final value");
            plot_y.drawCurve(time_cum, y_tracker).label("tracker");
            plot_y.drawCurve(time_cum, y_predictor).label("prediction");
            plot_y.xrange(0.0, time_cum.back());
            plot_y.yrange(0.0, 1.0);

            plot_convergence.drawCurve(time_cum, gamma_threshold).label("threshold");
            plot_convergence.drawCurve(time_cum, convergence_series).label("tot variation");
            plot_convergence.xrange(0.0, time_cum.back());
            float max_gamma = *max_element(convergence_series.begin(), convergence_series.end());
            plot_convergence.yrange(0.0, std::max(convergence_thresh, max_gamma));


            // show the plot in a pop-up window
            Figure fig = {{plot_t}, {plot_y}, {plot_convergence}};
            fig.size(1000, 1300);       // width, height
            fig.show();
            fig.save("time-y-gamma.pdf");

            // some cleanup
            time_intervals.clear();
            time_prediction.clear();
            convergence_series.clear();
            y_tracker.clear();
            y_predictor.clear();
            plot_t.clear();
            plot_t.cleanup();
            plot_y.clear();
            plot_y.cleanup();
            plot_convergence.clear();
            plot_convergence.cleanup();
            fig.cleanup();

            image_mutex.unlock();
        }
    }

    void resetTrial(){
        u_target = -304;
        v_target = -240;
        convergence_pts.clear();
        time_intervals.clear();
        time_prediction.clear();
        y_tracker.clear();
        y_predictor.clear();
        is_converged = false;
        convergence_series.clear();
    }

    bool respond(const Bottle &command, Bottle &reply) {
        string cmd=command.get(0).asString();
        if (cmd=="help") {
            reply.addVocab32(Vocab32::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- moveHome (bring robot back to home position)");
            reply.addString("- draw (draws tracker and prediction trajectories)");
            reply.addString("- printJointsPosition (print 7D vector containing joint values)");
            reply.addString("- printCartesianPose (print T_o_ee matrix)");
            reply.addString("- resetTrial");
            reply.addString("- quit");
        } else if (cmd=="printJointsPosition") {
            printJointsPosition(robot);
            reply.addString("printed q vector on main terminal!");
        } else if (cmd=="printCartesianPose") {
            printCartesianPose(robot);
            reply.addString("printed p vector on main terminal!");
        } else if (cmd=="moveHome") {
            try {
                go2JointPositions(robot, q_goal, 0.1);
                setRobotCollisionBehavior(robot);
                yInfo() << "Robot back to home position...";
            } catch (const franka::Exception& e) {
                std::cout << e.what() << std::endl;
                std::cout << "Running error recovery..." << std::endl;
                robot->automaticErrorRecovery();
                reply.addString("an error occurred during motion and automaticErrorRecovery has been launched!");
                return -1;
            }
            reply.addString("ack");
            reply.addString("Back home!");
        } else if (cmd=="draw") {
            createImage();
            reply.addString("ack");
            reply.addString("Draw completed!");
        } else if (cmd=="abilitateMotion") {
            can_move = !can_move;
            reply.addString("ack");
            if (can_move) {
                reply.addString("Motion enabled!");
            } else {
                reply.addString("Motion disabled!");
            }
        } else if (cmd=="resetTrial") {
            resetTrial();
            image_mutex.unlock();
            reply.addString("ack");
            reply.addString("Trial resetted!");
        } else {

            return RFModule::respond(command,reply);
        }
        return true;
    }

    virtual double getPeriod() {
        return period;
    }

    bool interruptModule() {
        return Thread::stop();
    }

    void onStop() {
        if (robot != nullptr)
            delete robot;
        prediction.close();
        handler_port.close();
    }

};


int main(int argc, char * argv[]) {
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "pandaCartController" );
    rf.setDefaultConfigFile( "panda_cart_controller.ini" );
    rf.configure( argc, argv );

    /* create the module */
    PandaCartesianController cart_controller;
    return cart_controller.runModule(rf);
}
