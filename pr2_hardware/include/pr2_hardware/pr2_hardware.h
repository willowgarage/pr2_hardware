#ifndef PR2_HARDWARE_PR2_HARDWARE_H
#define PR2_HARDWARE_PR2_HARDWARE_H

#include <pr2_mechanism_model/robot.h>
#include <hardware_interface/hardware_interface.h>

namespace pr2_hardware
{

class PR2Hardware : public pr2_mechanism_model::RobotState,
                    virtual public hardware_interface::HardwareInterface
{
public:
  PR2Hardware(pr2_mechanism_model::Robot* model) :
    pr2_mechanism_model::RobotState(model)
  {
    registerType(typeid(pr2_mechanism_model::RobotState).name());
  }
};

}

#endif // PR2_HARDWARE_PR2_HARDWARE_H
