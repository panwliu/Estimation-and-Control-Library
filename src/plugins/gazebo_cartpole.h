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

namespace gym_gazebo {

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
        //ignition::gazebo::Link cart_link{{kNullEntity}};

        int n_model_;
        std::string model_name_;
        std::string link_name_cart_;
        std::string link_name_pole_;
        std::string states_topic_;
        std::string commands_topic_;
        ignition::transport::Node node_;
        ignition::transport::Node::Publisher states_pub_;
        ignition::msgs::Float_V states_msg_;

        std::vector<ignition::gazebo::Entity> model_entities_;      //can't use ignition::gazebo::Entity *model_entities_ptr_;
        std::vector<ignition::gazebo::Entity> link_entities_cart_;
        std::vector<ignition::gazebo::Entity> link_entities_pole_;

        std::vector<ignition::gazebo::Model> models_;
        std::vector<ignition::gazebo::Link> links_cart_;
        std::vector<ignition::gazebo::Link> links_pole_;
};

}

#endif