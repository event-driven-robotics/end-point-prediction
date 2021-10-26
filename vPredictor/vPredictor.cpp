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
    bool save;
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
    vReadPort< vector<LabelledAE> > input_port;
    yarp::os::BufferedPort<yarp::os::Bottle> coordPort;
    RpcServer handler_port;

    BufferedPort<Bottle> output_port;

public:

    vPredictor() {}

    virtual bool configure(yarp::os::ResourceFinder& rf) {

        //set the module name used to name ports and the module period
        setName((rf.check("name", Value("/vPredictor")).asString()).c_str());
        period = rf.check("period", Value(1.0)).asDouble();
        reset_time = rf.check("resetTime", Value(1.0)).asDouble();

       if(!input_port.open(getName() + "/LAE:i")) {
            yError() << "Could not open input port";
            return false;
        }

        if(!LAE_output_port.open(getName() + "/LAE:o")) {
            yError() << "Could not open LAE output port";
            return false;
        }

        if(!output_port.open(getName() + "/tracker_prediction:o")) {
            yError() << "Could not open output_port";
            return false;
        }

        if(!coordPort.open(getName() + "/coords:i"))
            return false;

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
        input_scaler[0] = 0.0; input_scaler[1] = 0.1;
        input_scaler[2] = 0.0; input_scaler[3] = 304.0;
        input_scaler[4] = 0.0; input_scaler[5] = 240.0;

        // range of the features on which the model is trained
        prediction_scaler.resize(6);  //lower and upper bound for 3 features (t,x,y)
        prediction_scaler[0] = -1.0; prediction_scaler[1] =  0.0;
        prediction_scaler[2] = -304.0; prediction_scaler[3] =  304.0;
        prediction_scaler[4] = -240.0; prediction_scaler[5] =  240.0;

//        Network::connect("/COMtrackerRight/LAE:o", "/vPredictorRight/LAE:i");
        Network::connect("/COMtracker/coords:o", "/vPredictor/coords:i");
        
        //open rpc port
        handler_port.open(getName()+"/rpc:i");
        attach(handler_port);

        save = true;

        std::cout << "configure"<<std::endl;

        return Thread::start();
    }

    void run() {

        double x, y;
        double prev_t, dt = 0;

        Stamp yarpstamp;

        LabelledAE v;
        deque<LabelledAE> LAE_out_queue;

        //read some data to extract the t0
        const vector<LabelledAE> *q = input_port.read(yarpstamp);
        if(!q || Thread::isStopping()) return;
        prev_t = q->back().stamp;

        std::cout << "before loop"<<std::endl;

        while(true) {

            const vector<LabelledAE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            std::cout << "inside loop"<<std::endl;

            dt = q->back().stamp - prev_t;
            if(dt < 0) dt += vtsHelper::max_stamp;
            dt *= vtsHelper::tsscaler;
            prev_t = q->back().stamp;

            if (dt > reset_time) {
                model_stateful->reset_states();
                yWarning() << "Resetted LSTM memory after " << dt << " seconds";
                dt = 0.0;
            }

            yarp::os::Bottle* coordBottle = coordPort.read();
            x = coordBottle->get(0).asDouble();
            y = coordBottle->get(1).asDouble();

            // x = q->back().x;
            // y = q->back().y;

            //OPENCV stuff
            // add tracker data to output bottle
            Bottle &pred = output_port.prepare();
            pred.clear();
            pred.addDouble(dt);                     // dt between tracked points
            pred.addInt(x);                         // tracker x coordinate
            pred.addInt(y);                         // tracker y coordinate

            // scale the input
            scale(dt, x, y);

            // query the model (lines 133-137 = line 138)
//            fdeep::tensor t(fdeep::tensor_shape(1,3),0);
//            t.set(fdeep::tensor_pos(0,0), (float)scaled_input[0]);
//            t.set(fdeep::tensor_pos(0,1), (float)scaled_input[1]);
//            t.set(fdeep::tensor_pos(0,2), (float)scaled_input[2]);
//            auto result = model_stateful->predict_stateful({t});
            fdeep::tensors result = model_stateful->predict_stateful({fdeep::tensor(fdeep::tensor_shape(1,3), fdeep::float_vec{(float)scaled_input[0], (float)scaled_input[1], (float)scaled_input[2]})});
            std::vector<float> vec = result[0].to_vector();

            // rescale the output prediction
            inverse_scale(vec);

//            //measure milliseconds
//            yInfo() << std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t1).count();
            // std::cout << "dt = " << dt << "   X = " << x << "   Y = " << y;
            if (save)
                log_file << dt << " , " << x << " , " << y;

            // var - pred[i] is necessary because the output of the predictor is not the final position
            // but the shift between the latter and the current input
            x = x - prediction[1];
            // yInfo() << x;
            y = y - prediction[2];
            if (x < input_scaler[2]) { x = input_scaler[2]; }
            if (x > input_scaler[3]) { x = input_scaler[3]; }
            if (y < input_scaler[4]) { y = input_scaler[4]; }
            if (y > input_scaler[5]) { y = input_scaler[5]; }

            // std::cout << "   E.T.A. = " << prediction[0] << "   X = " << x << "   Y = " << y << std::endl;
            if (save)
                log_file << " , " << prediction[0] << " , " << x << " , " << y << std::endl;

            std::cout << "a"<<std::endl;

            // output LAE
            v.ID = 1;
            v.x = x;
            v.y = y;
            v.stamp = q->back().stamp;
            LAE_out_queue.push_back(v);
            LAE_output_port.write(LAE_out_queue, yarpstamp);
            LAE_out_queue.clear();

            std::cout << "b"<<std::endl;

            // add predictor data to output bottle
            pred.addDouble(-prediction[0]);         // e.t.a.
            pred.addInt(v.x);                       // predictor x coordinate
            pred.addInt(v.y);                       // predictor y coordinate
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
        input_port.close();
        LAE_output_port.close();
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
