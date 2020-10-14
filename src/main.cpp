// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <memory>
#include <string>
#include <cmath>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/pcl/Pcl.h>

#include "viewer.h"

/*************************************************************************************/
class Module: public yarp::os::RFModule {
    yarp::dev::PolyDriver driverGaze;
    yarp::dev::IGazeControl* igaze;
    double fov_h{0.};
    double view_angle{0.};
    bool stopping{false};

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> depthPort;
    yarp::os::RpcServer rpcPort;

    std::unique_ptr<viewer::Viewer> viewer;

    /*********************************************************************************/
    bool configure(yarp::os::ResourceFinder& rf) override {
        std::string object_model_file = rf.findFile("models/object/mesh.stl");

        // let's use the IGazeControl I/F to move the gaze
        // and acquire useful information
        yarp::os::Property optionsGaze;
        optionsGaze.put("device", "gazecontrollerclient");
        optionsGaze.put("remote", "/iKinGazeCtrl");
        optionsGaze.put("local", "/gaze");
        if (driverGaze.open(optionsGaze)) {
            driverGaze.view(igaze);
        } else {
            yError() << "Unable to connect to iKinGazeCtrl";
            return false;
        }

        // get camera intrinsics
        yarp::os::Bottle info;
        igaze->getInfo(info);
        auto intrinsics = info.find("camera_intrinsics_left").asList();
        fov_h = intrinsics->get(0).asDouble();
        auto w_2 = intrinsics->get(2).asDouble();
        view_angle = 2. * std::atan(w_2 / fov_h) * (180. / M_PI);

        imgPort.open("/img:i");
        depthPort.open("/depth:i");

        rpcPort.open("/service");
        attach(rpcPort);

        // block the eyes vergence to a predefined value
        // that will work with the pyshical robot
        igaze->blockEyes(5.);

        // look down at the table {azimuth, elevation, vergence}
        // (vergence is blocked but the service needs it anyway)
        igaze->lookAtAbsAnglesSync({0., -60., 5.});

        // wait until the movement is over
        igaze->waitMotionDone();

        // start the 3D viewer as the very last thing
        viewer = std::make_unique<viewer::Viewer>(10, 370, 350, 350);
        viewer->start();

        return true;
    }

    /*********************************************************************************/
    bool interruptModule() override {
        viewer->stop();
        imgPort.interrupt();
        depthPort.interrupt();
        stopping = true;
        return true;
    }

    /*********************************************************************************/
    bool close() override {
        driverGaze.close();
        imgPort.close();
        depthPort.close();
        rpcPort.close();
        return true;
    }

    /*********************************************************************************/
    bool updateModule() override {
        // updateModule() won't be executed because of the VTK thread
        return true;
    }

    /*********************************************************************************/
    bool respond(const yarp::os::Bottle& cmd,
                 yarp::os::Bottle& reply) override {
        if (cmd.get(0).asVocab() == yarp::os::Vocab::encode("go")) {
            reply.addInt(go());
        } else {
            // the parent class handles the "quit" command
            return yarp::os::RFModule::respond(cmd, reply);
        }
        return true;
    }

    /*********************************************************************************/
    bool explore_table(const double azimuth, const double elevation) {
        // FILL IN THE CODE
        return true;
    }

    /*********************************************************************************/
    auto acquire_scene() {
        // get fresh images
        auto* img = imgPort.read();
        auto* depth = depthPort.read();

        // interrupt sequence detected
        if ((img == nullptr) || (depth == nullptr)) {
            return std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>>(nullptr);
        }

        // get camera extrinsics
        // we need extrinsincs for expressing the point clouds
        // in the root frame rather than the eye frame,
        // so that we can then deal with the proper coordinates
        // system to finally determine the position of the object
        // on the table
        yarp::sig::Vector cam_x, cam_o;
        igaze->getLeftEyePose(cam_x, cam_o);
        auto Teye = yarp::math::axis2dcm(cam_o);
        Teye.setSubcol(cam_x, 0, 3);

        // when you aim to get the point cloud of the scene
        // just call the following:
        const auto w = depth->width();
        const auto h = depth->height();
        auto pc = std::make_shared<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>>();
        yarp::sig::Vector x{0., 0., 0., 1.};
        for (int v = 0; v < h; v++) {
            for (int u = 0; u < w; u++) {
                const auto rgb = (*img)(u, v);
                const auto d = (*depth)(u, v);
                
                if (d > 0.F) {
                    x[0] = d * (u - .5 * (w - 1)) / fov_h;
                    x[1] = d * (v - .5 * (h - 1)) / fov_h;
                    x[2] = d;
                    x = Teye * x;
                
                    pc->push_back(yarp::sig::DataXYZRGBA());
                    auto& p = (*pc)(pc->size() - 1);
                    p.x = (float)x[0];
                    p.y = (float)x[1];
                    p.z = (float)x[2];
                    p.r = rgb.r;
                    p.g = rgb.g;
                    p.b = rgb.b;
                    p.a = 255;
                }
            }
        }

        // 3D visualization
        viewer->setCamera({cam_x[0], cam_x[1], cam_x[2]},
                          {cam_x[0] + Teye(0, 2), cam_x[1] + Teye(1, 2), cam_x[2] + Teye(2, 2)},
                          {0., 0., 1.}, view_angle);
        viewer->showPointCloud(pc);

        return pc;
    }

    /*********************************************************************************/
    int go() {
        int k{-1};
        while ((k < 0) && !stopping) {
            // FILL IN THE CODE
            double azimuth{0.};
            double elevation{-60.};
            explore_table(azimuth, elevation);
            auto pc = acquire_scene();

            // FILL IN THE CODE
            k = 1;
        }
        
        // return here the position of the object on the table
        // shall be in {1, 2, 3, 4}
        return k;
    }
};


/*************************************************************************************/
int main(int argc, char *argv[]) {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    Module module;
    return module.runModule(rf);
}