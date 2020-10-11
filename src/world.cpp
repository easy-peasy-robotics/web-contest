// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <mutex>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Pose3.hh>

#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>

namespace gazebo {

/******************************************************************************/
class WorldHandler : public gazebo::WorldPlugin
{
    gazebo::physics::WorldPtr world;

    std::mutex mtx;
    struct ModelData {
        gazebo::physics::ModelPtr model{nullptr};
        int table_position{-1};
    };
    std::vector<ModelData> models;

    yarp::os::Port rpcPort;
    /**************************************************************************/
    class DataProcessor : public yarp::os::PortReader {
        WorldHandler* hdl;
        /**********************************************************************/
        bool read(yarp::os::ConnectionReader& connection) override {
            yarp::os::Bottle cmd;
            cmd.read(connection);
            auto* returnToSender = connection.getWriter();
            if (returnToSender != nullptr) {
                yarp::os::Bottle rep;
                std::lock_guard<std::mutex> lck(hdl->mtx);
                if (cmd.get(0).asVocab() == yarp::os::Vocab::encode("shuffle")) {
                    rep.addVocab(yarp::os::Vocab::encode("ack"));
                    rep.addInt(hdl->shuffle(true));
                } else {
                    rep.addVocab(yarp::os::Vocab::encode("nack"));
                }
                rep.write(*returnToSender);
            }
            return true;
        }
    public:
        /**********************************************************************/
        DataProcessor(WorldHandler* hdl_) : hdl(hdl_) { } 
    } processor;
    friend class DataProcessor;

    /**************************************************************************/
    int shuffle(const bool move_first = false) {
        std::vector<size_t> idx(models.size());
        std::iota(std::begin(idx), std::end(idx), 0);
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::shuffle(std::begin(idx), std::end(idx), std::default_random_engine(seed));

        std::vector<ignition::math::Pose3d> poses;
        std::vector<int> table_positions;
        for (auto& m:models) {
            auto pose = m.model->WorldPose();
            if (move_first) {
                pose.Pos().X() += -.3;
                m.model->SetWorldPose(pose);
            }
            poses.push_back(pose);
            table_positions.push_back(m.table_position);
        }

        int k{-1};
        std::random_device rnd_device;
        std::mt19937 mersenne_engine(rnd_device());
        std::uniform_real_distribution<double> rot(-M_PI, M_PI);
        for (size_t i = 0; i < idx.size(); i++) {
            const auto j = idx[i];
            const auto ang = rot(mersenne_engine) / 2.;

            const auto p = poses[j].Pos();
            const auto q = poses[j].Rot() * ignition::math::Quaterniond(std::cos(ang), 0., 0., std::sin(ang));
            ignition::math::Pose3d pose_i(p.X() + .3, p.Y(), p.Z(), q.W(), q.X(), q.Y(), q.Z());

            auto& m = models[i];
            m.model->SetWorldPose(pose_i);
            m.table_position = table_positions[j];
            if (m.model->GetName() == "object") {
                k = m.table_position;
            }
        }

        return k;
    }

public:
    /**************************************************************************/
    WorldHandler() : processor(this) { }
    
    /**************************************************************************/
    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr) override {
        this->world = world;
        models.push_back({world->ModelByName("fallback-1"), 1});
        models.push_back({world->ModelByName("fallback-2"), 2});
        models.push_back({world->ModelByName("fallback-3"), 3});
        models.push_back({world->ModelByName("object"), 4});

        shuffle();

        rpcPort.setReader(processor);
        rpcPort.open("/world/rpc");
    }

    /**************************************************************************/
    virtual ~WorldHandler() {
        if (rpcPort.isOpen()) {
            rpcPort.close();
        }
    }
};

}

GZ_REGISTER_WORLD_PLUGIN(gazebo::WorldHandler)
