#include "gazebo_cartpole.h"

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>

#include <ignition/math/Vector3.hh>

#include <ignition/msgs/float_v.pb.h>

namespace gym_gazebo{

GazeboCartpole::GazeboCartpole()
{}

GazeboCartpole::~GazeboCartpole()
{}

void GazeboCartpole::Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr)
{
    std::cout<<"!!!!!Enter GazeboCartpole"<<std::endl;

    auto sdfClone = _sdf->Clone();
    n_model_ = sdfClone->Get<int>("model_amount");
    model_name_ = sdfClone->Get<std::string>("model_name");
    link_name_cart_ = sdfClone->Get<std::string>("link_name_cart");
    link_name_pole_ = sdfClone->Get<std::string>("link_name_pole");
    states_topic_ = sdfClone->Get<std::string>("states_topic");
    commands_topic_ = sdfClone->Get<std::string>("commands_topic");

    std::cout<<n_model_<<std::endl;
    std::cout<<model_name_<<std::endl;
    std::cout<<link_name_cart_<<std::endl;
    std::cout<<link_name_pole_<<std::endl;

    std::string tmp_mdoel_name;
    for (int k=0; k<n_model_; k++)      // get the models, links and their entities
    {
        tmp_mdoel_name = model_name_ + std::to_string(k);
        std::cout<<tmp_mdoel_name<<std::endl;
        model_entities_.push_back( 
            _ecm.EntityByComponents(ignition::gazebo::components::Name(tmp_mdoel_name)) );
        std::cout<<"Entity is "<<model_entities_[k]<<std::endl;

        auto tmp = _ecm.ChildrenByComponents(
            model_entities_[k], ignition::gazebo::components::Link(),ignition::gazebo::components::Name(link_name_cart_) );
        link_entities_cart_.push_back(tmp[0]);

        auto tmp2 = _ecm.ChildrenByComponents(
            model_entities_[k], ignition::gazebo::components::Link(),ignition::gazebo::components::Name(link_name_pole_) );
        link_entities_pole_.push_back(tmp2[0]);

        models_.push_back(ignition::gazebo::Model(model_entities_[k]));
        links_cart_.push_back(ignition::gazebo::Link(link_entities_cart_[k]));
        links_pole_.push_back(ignition::gazebo::Link(link_entities_pole_[k]));
    }

    for (int k=0; k<n_model_; k++)      // add components to links s.t. force can be applied, and pose, velocity can be extracted
    {
        _ecm.CreateComponent(link_entities_cart_[k], ignition::gazebo::components::WorldPose());
        _ecm.CreateComponent(link_entities_cart_[k], ignition::gazebo::components::WorldLinearVelocity());

        _ecm.CreateComponent(link_entities_pole_[k], ignition::gazebo::components::WorldPose());
        _ecm.CreateComponent(link_entities_pole_[k], ignition::gazebo::components::WorldAngularVelocity());
    }

    states_pub_ = node_.Advertise<ignition::msgs::Float_V>(states_topic_);
    node_.Subscribe(commands_topic_, &GazeboCartpole::CommandTopicCB, this);

/*
    std::cout<<"Entity is "<<_entity<<std::endl;
    auto a = _ecm.EntityByComponents(ignition::gazebo::components::Name("ground_plane"));
    auto b = _ecm.ChildrenByComponents(_entity, ignition::gazebo::components::Model());
    std::cout<<"Model Entity1 is "<<a<<std::endl;
    std::cout<<"Model Entity2 is "<<b.size()<<std::endl;
    auto model = ignition::gazebo::Model(_entity);
    std::cout<<"Model is "<<model.Name(_ecm)<<std::endl;
    auto c = _ecm.EntitiesByComponents(ignition::gazebo::components::Model());
    std::cout<<"Model Entity3 is "<<c.size()<<std::endl;
    auto d = _ecm.EntityByComponents(ignition::gazebo::components::Name("cartpole0"));
    auto d_cart = _ecm.ChildrenByComponents(_entity, ignition::gazebo::components::Link(),ignition::gazebo::components::Name("cart"));
    ignition::gazebo::Entity d_dart0 = d_cart[0];
    auto cart_link = ignition::gazebo::Link(d_dart0);
    std::cout<<"Model d_cart is "<<d_cart.size()<<std::endl;
    cart_link.AddWorldForce(_ecm, ignition::math::Vector3d(10,0,0));
    _ecm.CreateComponent(d_dart0, ignition::gazebo::components::WorldPose());
    _ecm.CreateComponent(d_dart0,ignition::gazebo::components::WorldLinearVelocity());*/
}

void GazeboCartpole::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm)
{
    
    links_cart_[0].AddWorldForce(_ecm, ignition::math::Vector3d(-0.1,0,0));     // make sure the required components are added first, what components are required can be looked into the function

}

void GazeboCartpole::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm)
{
    states_msg_.clear_data();
    states_msg_.add_data(_info.simTime.count()/1e9);        // simulation time
    states_msg_.add_data(n_model_);                         // number of models
    states_msg_.add_data(4);                                // number of states

    for(int k=0; k<n_model_; k++)
    {
        auto position = links_cart_[k].WorldPose(_ecm)->Pos();
        auto vel = links_cart_[k].WorldLinearVelocity(_ecm);
        auto euler = links_pole_[k].WorldPose(_ecm)->Rot().Euler();
        auto omega = links_pole_[k].WorldAngularVelocity(_ecm);
        // auto vel = links_[0].WorldLinearVelocity(_ecm, ignition::math::Vector3d(0,0,0));     // see source file at ignition gazebo src/Link.cc, three components are required
        //auto vel2 = _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(link_entities_cart_[0]);

        /*
        std::cout<<"Position is     "<<"["<<position.X()<<", "<<position.Y()<<", "<<position.Z()<<"]"<<std::endl;
        std::cout<<"Velocity is     "<<"["<<vel->X()<<", "<<vel->Y()<<", "<<vel->Z()<<"]"<<std::endl;
        std::cout<<"Angle is        "<<"["<<euler.X()*180/3.14<<", "<< euler.Y()<<", "<<euler.Z()*180/3.14<<"]"<<std::endl;
        std::cout<<"Angular rate is "<<"["<<omega->X()*180/3.14<<", "<<omega->Y()<<", "<<omega->Z()*180/3.14<<"]"<<std::endl;
        //std::cout<<"Velocity2 is "<<"["<<vel2->Data()<<"]"<<std::endl;
        std::cout<<"sim time is     "<<_info.simTime.count()/1e9<<std::endl;
        */
        
        states_msg_.add_data(position.X()); // x position
        states_msg_.add_data(vel->X());     // x velocity
        states_msg_.add_data(euler.Y());    // pitch angle
        states_msg_.add_data(omega->Y());   // pitch rate
    }
    
    if(!_info.paused)
        states_pub_.Publish(states_msg_);
}

void GazeboCartpole::CommandTopicCB(const ignition::msgs::Float_V &_msg)
{
    auto data = _msg.data();
    float force[3] = {data.Get(4), data.Get(6), data.Get(8)};

    for(int k=0; k<9; k++)
    {
        std::cout<<data.Get(k)<<" ";
    }
    std::cout<<std::endl;
}

}

IGNITION_ADD_PLUGIN(gym_gazebo::GazeboCartpole,
                    ignition::gazebo::System,
                    gym_gazebo::GazeboCartpole::ISystemConfigure,
                    gym_gazebo::GazeboCartpole::ISystemPreUpdate,
                    gym_gazebo::GazeboCartpole::ISystemPostUpdate)

