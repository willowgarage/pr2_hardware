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
  PR2Hardware(pr2_hardware_interface::HardwareInterface* hw)
    : robot_model_(hw)
  {}


  ~PR2Hardware()
  {
    if (robot_state_) 
      delete robot_state_;
  }


  bool initXml(TiXmlElement* config)
  {
    // create robot
    if (!robot_model_.initXml(config))
    {
      ROS_ERROR("Failed to initialize pr2 mechanism model");
      return false;
    }
    robot_state_ = new pr2_mechanism_model::RobotState(&robot_model_);
    
    // initialize motor state
    motors_previously_halted_ = robot_state_->isHalted();
    reset_controllers = false;
    
    // register interfaces
    typedef std::map<std::string, pr2_mechanism_model::JointState*> JointStateMap;
    for(JointStateMap::iterator it = robot_state_->joint_states_map_.begin();
	it != robot_state_->joint_states_map_.end(); ++it)
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
    registerInterface(robot_state_);

    return true;
  }
  

  void read()
  {
    robot_state_->propagateActuatorPositionToJointPosition();
    robot_state_->zeroCommands();
    
    // Restart all running controllers if motors are re-enabled
    reset_controllers = !robot_state_->isHalted() && motors_previously_halted_;
    motors_previously_halted_ = robot_state_->isHalted();
  }


  void write()
  {
    robot_state_->enforceSafety();
    robot_state_->propagateJointEffortToActuatorEffort();
  }


  bool reset_controllers;

  pr2_mechanism_model::Robot robot_model_;
  pr2_mechanism_model::RobotState* robot_state_;

private:
  bool motors_previously_halted_;
  hardware_interface::JointStateInterface  joint_state_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
};

}

#endif // PR2_HARDWARE_PR2_HARDWARE_H
