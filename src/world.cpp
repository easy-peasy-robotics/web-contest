// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <iterator>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
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
    gazebo::event::ConnectionPtr renderer_connection;

    std::mutex mtx;
    struct ModelData {
        gazebo::physics::ModelPtr model{nullptr};
        ignition::math::Pose3d pose;
    };
    std::unordered_map<std::string, ModelData> models; 

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
                if ((cmd.size() > 1) && (cmd.get(0).asVocab() == yarp::os::Vocab::encode("pose"))) {
                    const auto& it = hdl->models.find(cmd.get(1).asString());
                    if (it != hdl->models.end()) {
                        const auto& p = it->second.pose.Pos();
                        const auto& q = it->second.pose.Rot();
                        rep.addVocab(yarp::os::Vocab::encode("ack"));
                        rep.addDouble(p.X());
                        rep.addDouble(p.Y());
                        rep.addDouble(p.Z());
                        rep.addDouble(q.W());
                        rep.addDouble(q.X());
                        rep.addDouble(q.Y());
                        rep.addDouble(q.Z());
                    } else {
                        rep.addVocab(yarp::os::Vocab::encode("nack"));
                    }
                } else if (cmd.get(0).asVocab() == yarp::os::Vocab::encode("names")) {
                    rep.addVocab(yarp::os::Vocab::encode("ack"));
                    for (auto& m:hdl->models) {
                        rep.addString(m.first);
                    }
                } else if (cmd.get(0).asVocab() == yarp::os::Vocab::encode("shuffle")) {
                    hdl->shuffle(true);
                    rep.addVocab(yarp::os::Vocab::encode("ack"));
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
    void onWorld() {
        std::lock_guard<std::mutex> lck(mtx);
        for (auto& m:models) {
            m.second.pose = m.second.model->WorldPose();
        }
    }

    /**************************************************************************/
    void shuffle(const bool move_first = false) {
        std::vector<unsigned int> idx(models.size());
        std::iota(std::begin(idx), std::end(idx), 0);
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::shuffle(std::begin(idx), std::end(idx), std::default_random_engine(seed));

        std::vector<ignition::math::Pose3d> poses;
        for (auto& m:models) {
            auto pose = m.second.model->WorldPose();
            if (move_first) {
                pose.Pos().X() += -.3;
                m.second.model->SetWorldPose(pose);
            }
            poses.push_back(pose);
        }

        std::random_device rnd_device;
        std::mt19937 mersenne_engine(rnd_device());
        std::uniform_real_distribution<double> rot(-M_PI, M_PI);
        for (unsigned int i = 0; i < idx.size(); i++) {
            const auto j = idx[i];
            const auto ang = rot(mersenne_engine) / 2.;

            const auto p = poses[j].Pos();
            const auto q = poses[j].Rot() * ignition::math::Quaterniond(std::cos(ang), 0., 0., std::sin(ang));
            ignition::math::Pose3d pose_i(p.X() + .3, p.Y(), p.Z(), q.W(), q.X(), q.Y(), q.Z());
            
            auto m_i = models.begin();
            std::advance(m_i, i);
            m_i->second.model->SetWorldPose(pose_i);
        }
    }

public:
    /**************************************************************************/
    WorldHandler() : processor(this) { }
    
    /**************************************************************************/
    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr) override {
        this->world = world;
        for (unsigned int i = 0; i < world->ModelCount(); i++) {
            auto model = world->ModelByIndex(i);
            auto name = model->GetName();
            if ((name != "icub_head_epr") && (name != "ground_plane") && (name != "table")) {
                ModelData m;
                m.model = model;
                models[name] = m;
            }
        }

        shuffle();

        rpcPort.setReader(processor);
        rpcPort.open("/world/rpc");

        auto bind = std::bind(&WorldHandler::onWorld, this);
        renderer_connection = gazebo::event::Events::ConnectWorldUpdateBegin(bind);
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
