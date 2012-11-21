#ifndef PR2_HARDWARE_PR2_HARDWARE_H
#define PR2_HARDWARE_PR2_HARDWARE_H

#include <pr2_mechanism_model/robot.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace pr2_hardware
{

class PR2Hardware : public pr2_mechanism_model::RobotState,
                    virtual public hardware_interface::HardwareInterface,
                    public hardware_interface::EffortJointInterface
{
public:
  PR2Hardware(pr2_mechanism_model::Robot* model) :
    pr2_mechanism_model::RobotState(model)
  {
    registerType(typeid(pr2_mechanism_model::RobotState).name());
  }

  std::vector<std::string> getJointNames() const
  {
    std::vector<std::string> out;
    std::map<std::string, pr2_mechanism_model::JointState*>::const_iterator it;
    for(it = joint_states_map_.begin(); it != joint_states_map_.end(); it++)
      out.push_back((*it).first);
    return out;
  }

  double* getEffortCommand(const std::string& name)
  {
    pr2_mechanism_model::JointState* js = pr2_mechanism_model::RobotState::getJointState(name);
    if (js)
      return &js->commanded_effort_;
    return NULL;
  }

  const double* getPosition(const std::string &name) const
  {
    const pr2_mechanism_model::JointState* js = pr2_mechanism_model::RobotState::getJointState(name);
    if (js)
      return &js->position_;
    return NULL;
  }

  const double* getVelocity(const std::string &name) const
  {
    const pr2_mechanism_model::JointState* js = pr2_mechanism_model::RobotState::getJointState(name);
    if (js)
      return &js->velocity_;
    return NULL;
  }

  const double* getEffort(const std::string &name) const
  {
    const pr2_mechanism_model::JointState* js = pr2_mechanism_model::RobotState::getJointState(name);
    if (js)
      return &js->measured_effort_;
    return NULL;
  }
};

}

#endif // PR2_HARDWARE_PR2_HARDWARE_H
