#include "gazebo_cartpole.h"

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>

#include <ignition/math/Vector3.hh>

#include <ignition/msgs/float_v.pb.h>

namespace ecl{

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
    model_name_ = sdfClone->Get<std::string>("model_name");
    cart_link_name_ = sdfClone->Get<std::string>("link_name_cart");
    pole_link_name_ = sdfClone->Get<std::string>("link_name_pole");
    states_topic_ = sdfClone->Get<std::string>("states_topic");
    commands_topic_ = sdfClone->Get<std::string>("commands_topic");
    model_id_ = sdfClone->Get<int>("model_id");
    msg_len_ = sdfClone->Get<int>("message_length");

    std::cout<<model_name_<<std::endl;
    std::cout<<cart_link_name_<<std::endl;
    std::cout<<pole_link_name_<<std::endl;
    std::cout<<model_id_<<"\t"<<msg_len_<<std::endl;

    //--------------- get entities ----------------
    model_entity_ = _ecm.EntityByComponents(ignition::gazebo::components::Name(model_name_));
    auto tmp = _ecm.ChildrenByComponents(
        model_entity_, ignition::gazebo::components::Link(),ignition::gazebo::components::Name(cart_link_name_) );
    cart_link_entity_ = tmp[0];
    auto tmp2 = _ecm.ChildrenByComponents(
        model_entity_, ignition::gazebo::components::Link(),ignition::gazebo::components::Name(pole_link_name_) );
    pole_link_entity_ = tmp2[0];

    std::cout<<"Model entity: "<<model_entity_<<std::endl;
    std::cout<<"Cart link entity: "<<cart_link_entity_<<std::endl;
    std::cout<<"Pole link entity: "<<pole_link_entity_<<std::endl;

    model_ = ignition::gazebo::Model(model_entity_);
    cart_link_ = ignition::gazebo::Link(cart_link_entity_);
    pole_link_ = ignition::gazebo::Link(pole_link_entity_);

    //--------------- get model, links, coponents ----------------
    _ecm.CreateComponent(cart_link_entity_, ignition::gazebo::components::WorldPose());
    _ecm.CreateComponent(cart_link_entity_, ignition::gazebo::components::WorldLinearVelocity());
    _ecm.CreateComponent(pole_link_entity_, ignition::gazebo::components::WorldPose());
    _ecm.CreateComponent(pole_link_entity_, ignition::gazebo::components::WorldAngularVelocity());

    //--------------- publish. subscribe topics ----------------
    states_pub_ = node_.Advertise<ignition::msgs::Float_V>(states_topic_);
    node_.Subscribe(commands_topic_, &GazeboCartpole::CommandTopicCB, this);

}

void GazeboCartpole::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm)
{
    
    cart_link_.AddWorldForce(_ecm, ignition::math::Vector3d(command_force_,0,0));     // make sure the required components are added first, what components are required can be looked into the function

}

void GazeboCartpole::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    states_msg_.clear_data();
    states_msg_.add_data(model_id_);                                // id
    states_msg_.add_data(1);                                // type: 1-state data
    states_msg_.add_data(_info.simTime.count()/1e9);        // simulation time
    
    auto position = cart_link_.WorldPose(_ecm)->Pos();
    auto vel = cart_link_.WorldLinearVelocity(_ecm);
    auto euler = pole_link_.WorldPose(_ecm)->Rot().Euler();
    auto omega = pole_link_.WorldAngularVelocity(_ecm);
    
    /*std::cout<<"Position is     "<<"["<<position.X()<<", "<<position.Y()<<", "<<position.Z()<<"]"<<std::endl;
    std::cout<<"Velocity is     "<<"["<<vel->X()<<", "<<vel->Y()<<", "<<vel->Z()<<"]"<<std::endl;
    std::cout<<"Angle is        "<<"["<<euler.X()*180/3.14<<", "<< euler.Y()<<", "<<euler.Z()*180/3.14<<"]"<<std::endl;
    std::cout<<"Angular rate is "<<"["<<omega->X()*180/3.14<<", "<<omega->Y()<<", "<<omega->Z()*180/3.14<<"]"<<std::endl;
    //std::cout<<"Velocity2 is "<<"["<<vel2->Data()<<"]"<<std::endl;
    std::cout<<"sim time is     "<<_info.simTime.count()/1e9<<std::endl;*/
    
    states_msg_.add_data(position.X()); // x position
    states_msg_.add_data(vel->X());     // x velocity
    states_msg_.add_data(euler.Y());    // pitch angle
    states_msg_.add_data(omega->Y());   // pitch rate
    
    for (int j=0; j<msg_len_-7; j++)
        states_msg_.add_data(0);

    states_pub_.Publish(states_msg_);
}

void GazeboCartpole::CommandTopicCB(const ignition::msgs::Float_V &_msg)
{
    auto data = _msg.data();
    int id = data.Get(0);
    int type = data.Get(1);
    float t = data.Get(2);
    command_force_ = data.Get(3);
    //std::cout<<command_force_<<std::endl;

    /*for(int k=0; k<msg_len_; k++)
    {
        std::cout<<data.Get(k)<<" ";
    }
    std::cout<<std::endl;*/
}

}

IGNITION_ADD_PLUGIN(ecl::GazeboCartpole,
                    ignition::gazebo::System,
                    ecl::GazeboCartpole::ISystemConfigure,
                    ecl::GazeboCartpole::ISystemPreUpdate,
                    ecl::GazeboCartpole::ISystemPostUpdate)

