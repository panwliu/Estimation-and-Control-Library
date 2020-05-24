#ifndef GYM_GAZEBO_GAZEBOCARTPOLE_H_
#define GYM_GAZEBO_GAZEBOCARTPOLE_H_

#include <iostream>

#include <ignition/gazebo/System.hh>        // provide ignition::gazebo::xxx
#include <ignition/plugin/Register.hh>      // provide IGNITION_ADD_PLUGIN
#include <ignition/gazebo/EntityComponentManager.hh>

#include <ignition/gazebo/components/Name.hh>   // provide ignition::gazebo::components::Name()
#include <ignition/gazebo/components/Model.hh>  // provide ignition::gazebo::components::Model()
#include <ignition/gazebo/components/Link.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>

#include <ignition/transport/Node.hh>

namespace ecl {

class GazeboCartpole:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
{
    public:
        GazeboCartpole();
        ~GazeboCartpole() override;
        void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;
        void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
        void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
        void CommandTopicCB(const ignition::msgs::Float_V &_msg);

    private:
        std::string model_name_;
        std::string cart_link_name_;
        std::string pole_link_name_;
        std::string states_topic_;
        std::string commands_topic_;
        ignition::transport::Node node_;
        ignition::transport::Node::Publisher states_pub_;
        ignition::msgs::Float_V states_msg_;
        int model_id_;
        int msg_len_;

        ignition::gazebo::Entity model_entity_;
        ignition::gazebo::Entity cart_link_entity_;
        ignition::gazebo::Entity pole_link_entity_;

        ignition::gazebo::Model model_;
        ignition::gazebo::Link cart_link_;
        ignition::gazebo::Link pole_link_;

        float command_force_ = 0;
};

}

#endif