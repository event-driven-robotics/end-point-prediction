#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <event-driven/all.h>
#include <fdeep/fdeep.hpp>      // put "set(CMAKE_CXX_STANDARD 14)" in the CMake!!!
#include <deque>


using namespace ev;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


// ************************* vPREDICTOR ************************* //
class vPredictor : public RFModule, public Thread {

private:

    float period, reset_time;
    bool save, start;
    string log_path;
    ofstream log_file;

    string model_path;

    fdeep::model *model_stateful;

    yarp::sig::Vector scaled_input;
    yarp::sig::Vector prediction;
    yarp::sig::Vector new_range;
    yarp::sig::Vector input_scaler;
    yarp::sig::Vector prediction_scaler;

    vWritePort LAE_output_port;
    RpcServer handler_port;

    BufferedPort<Bottle> input_port;
    BufferedPort<Bottle> output_port;

public:

    vPredictor() {}

    virtual bool configure(yarp::os::ResourceFinder& rf) {

        //set the module name used to name ports and the module period
        setName((rf.check("name", Value("/vPredictor")).asString()).c_str());
        period = rf.check("period", Value(1.0)).asFloat64();
        reset_time = rf.check("resetTime", Value(1.0)).asFloat64();

        if(!LAE_output_port.open(getName() + "/LAE:o")) {
            yError() << "Could not open LAE output port";
            return false;
        }

        if(!output_port.open(getName() + "/tracker_prediction:o")) {
            yError() << "Could not open output_port";
            return false;
        }

        if(!input_port.open(getName() + "/tracker:i")) {
            return false;
        }

        // get current file path
        string file_path = __FILE__;
        string dir_path = file_path.substr(0, file_path.rfind("\\"));
        dir_path = dir_path.substr(0, dir_path.length() - 14); // -14 to remove "vPredictor.cpp"

        model_path = rf.check("modelPath", Value(dir_path)).asString();
        try{
            model_stateful = new fdeep::model(fdeep::load_model(model_path + "fdeep_model.json"));
        }
        catch (...)
        {
            yError() << "An error occurred while loading the LSTM model!";
            return 1;
        }

        // initialise coordinates vector
        scaled_input.resize(3, 0.0);
        prediction.resize(3, 0.0);

        // range used during model training for the activation function
        new_range.resize(2);
        new_range[0] = -1.0;
        new_range[1] =  1.0;

        // range of the features on which the model is trained
        input_scaler.resize(6);  //lower and upper bound for 3 features (t,x,y)
        input_scaler[0] = 0.0; input_scaler[1] = 0.01;
        input_scaler[2] = 0.0; input_scaler[3] = 304.0;
        input_scaler[4] = 0.0; input_scaler[5] = 240.0;

        // range of the features on which the model is trained
        prediction_scaler.resize(6);  //lower and upper bound for 3 features (t,x,y)
        prediction_scaler[0] = -2.0; prediction_scaler[1] =  0.0;
        prediction_scaler[2] = -304.0; prediction_scaler[3] =  304.0;
        prediction_scaler[4] = -240.0; prediction_scaler[5] =  240.0;

        //open rpc port
        handler_port.open(getName()+"/rpc:i");
        attach(handler_port);

        save = true;

        return Thread::start();
    }

    void run() {

        int counter = 0;

        double x, y, x_pred, y_pred, t_curr, prev_t, dt;

        double yarpstamp;

        LabelledAE v;
        deque<LabelledAE> LAE_out_queue;

        yarp::os::Bottle* coordBottle = input_port.read();
        x = coordBottle->get(0).asFloat64();
        y = coordBottle->get(1).asFloat64();
        prev_t = coordBottle->get(2).asFloat64();

        while(true) {

            yarp::os::Bottle* coordBottle = input_port.read();

//            yInfo() << "Spatial delta between points:  " << sqrt(pow(x- coordBottle->get(0).asFloat64(), 2) + pow(y - coordBottle->get(1).asFloat64(), 2));

            x = coordBottle->get(0).asFloat64();
            y = coordBottle->get(1).asFloat64();              // remember the 240!!!
            t_curr = coordBottle->get(2).asFloat64();
            yarpstamp = coordBottle->get(3).asFloat64();

            dt = t_curr - prev_t;
            if(dt < 0) dt += vtsHelper::max_stamp;//)*vtsHelper::tstosecs();
            dt *= vtsHelper::tsscaler;
            prev_t = t_curr;

            if (dt > reset_time) {
                model_stateful->reset_states();
                yWarning() << "Resetted LSTM memory after " << dt << " seconds";
                dt = 0.0;
            }

            //OPENCV stuff
            // add tracker data to output bottle
            Bottle &pred = output_port.prepare();
            pred.clear();
            pred.addFloat64(dt);                        // dt between tracked points
            pred.addFloat64(x);                         // tracker x coordinate
            pred.addFloat64(y);                         // tracker y coordinate

            yInfo() << "dt = " << dt  << "    x = " << x  << "    y = " << y;

            x_pred = x;
            y_pred = 240 - y;

            // scale the input
            scale(dt, x_pred, y_pred);

            // query the model (lines 177-183 = line 184)
/*
            fdeep::tensor t(fdeep::tensor_shape(1,3),0);
            t.set(fdeep::tensor_pos(0,0), (float)scaled_input[0]);
            t.set(fdeep::tensor_pos(0,1), (float)scaled_input[1]);
            t.set(fdeep::tensor_pos(0,2), (float)scaled_input[2]);
            auto result = model_stateful->predict_stateful({t});
*/
            fdeep::tensors result = model_stateful->predict_stateful({fdeep::tensor(fdeep::tensor_shape(1,3), fdeep::float_vec{(float)scaled_input[0], (float)scaled_input[1], (float)scaled_input[2]})});
            std::vector<float> vec = result[0].to_vector();

            // rescale the output prediction
            inverse_scale(vec);

            if (save)
                log_file << dt << " , " << x_pred << " , " << y_pred;

            // var - pred[i] is necessary because the output of the predictor is not the final position
            // but the shift between the latter and the current input
            x_pred = x_pred - prediction[1];
            y_pred = y_pred - prediction[2];
            if (x_pred < input_scaler[2]) { x_pred = input_scaler[2]; }
            if (x_pred > input_scaler[3]) { x_pred = input_scaler[3]; }
            if (y_pred < input_scaler[4]) { y_pred = input_scaler[4]; }
            if (y_pred > input_scaler[5]) { y_pred = input_scaler[5]; }

            // std::cout << "   E.T.A. = " << prediction[0] << "   X = " << x << "   Y = " << y << std::endl;
            if (save)
                log_file << " , " << prediction[0] << " , " << x_pred << " , " << y_pred << std::endl;

            // output LAE
            v.ID = 1;
            v.x = x_pred;
            v.y = 240 - y_pred;              // remember the 240!!!
            v.stamp = t_curr;
            Stamp stamp = Stamp(counter, yarpstamp);
            LAE_out_queue.push_back(v);
            LAE_output_port.write(LAE_out_queue, stamp);
            LAE_out_queue.clear();
            counter++;

            // add predictor data to output bottle
            pred.addFloat64(-prediction[0]);          // e.t.a.
            pred.addFloat64(x_pred);                  // predictor x coordinate
            pred.addFloat64(240 - y_pred);                  // predictor y coordinate
            output_port.write();
        }
    }

    void scale(float t, float x, float y) {
        scaled_input[0] = (((t-input_scaler[0])*(new_range[1]-new_range[0]))/(input_scaler[1]-input_scaler[0]))+new_range[0];
        scaled_input[1] = (((x-input_scaler[2])*(new_range[1]-new_range[0]))/(input_scaler[3]-input_scaler[2]))+new_range[0];;
        scaled_input[2] = (((y-input_scaler[4])*(new_range[1]-new_range[0]))/(input_scaler[5]-input_scaler[4]))+new_range[0];;
    }

    void inverse_scale(std::vector<float> output) {
        float temp1, temp2, temp3;
        temp1 = (((output[0]-new_range[0])*(prediction_scaler[1]-prediction_scaler[0]))/(new_range[1]-new_range[0]))+prediction_scaler[0];
        temp2 = (((output[1]-new_range[0])*(prediction_scaler[3]-prediction_scaler[2]))/(new_range[1]-new_range[0]))+prediction_scaler[2];
        temp3 = (((output[2]-new_range[0])*(prediction_scaler[5]-prediction_scaler[4]))/(new_range[1]-new_range[0]))+prediction_scaler[4];
        prediction[0] = temp1;
        prediction[1] = temp2;
        prediction[2] = temp3;
    }

    //rpc respond function
    bool respond(const Bottle &command, Bottle &reply) {
        string cmd=command.get(0).asString();
        if (cmd=="open") {
            save = true;
            log_path = command.get(1).asString();
            yInfo() << "file opened in "<< log_path;
            log_file.open(log_path);
            reply.addString("ok");
        } else if (cmd=="close") {
            save = false;
            log_file.close();
            reply.addString("ok");
        } else {
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        }
        return true;
    }

    virtual bool updateModule() {
        return Thread::isRunning();
    }

    virtual double getPeriod() {
        return period;
    }

    bool interruptModule() {
        return Thread::stop();
    }

    void onStop() {
        delete model_stateful;
        LAE_output_port.close();
        input_port.close();
        output_port.close();
    }
};


// ***************************** MAIN ****************************** //
int main(int argc, char * argv[])
{
    // initialize yarp network
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure( argc, argv );

    // create the module
    vPredictor predictor;
    return predictor.runModule(rf);
}
