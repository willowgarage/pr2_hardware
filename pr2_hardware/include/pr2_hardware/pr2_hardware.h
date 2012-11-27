#ifndef PR2_HARDWARE_PR2_HARDWARE_H
#define PR2_HARDWARE_PR2_HARDWARE_H

#include <pr2_mechanism_model/robot.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>


namespace pr2_hardware
{

class PR2Hardware : public hardware_interface::RobotHW
{
public:
  PR2Hardware(pr2_mechanism_model::Robot* model) :
    robot_state_(model)
  {
    typedef std::map<std::string, pr2_mechanism_model::JointState*> JointStateMap;
    for(JointStateMap::iterator it = robot_state_.joint_states_map_.begin();
        it != robot_state_.joint_states_map_.end(); ++it)
    {
      joint_state_interface_.registerJoint( it->first,
                                           &it->second->position_,
                                           &it->second->velocity_,
                                           &it->second->measured_effort_);
      effort_joint_interface_.registerJoint(joint_state_interface_.getJointStateHandle(it->first),
                                            &it->second->commanded_effort_);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&robot_state_);
  }

  pr2_mechanism_model::RobotState robot_state_;
  hardware_interface::JointStateInterface  joint_state_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
};

}

#endif // PR2_HARDWARE_PR2_HARDWARE_H
