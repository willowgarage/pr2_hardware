////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////


#include <pr2_hardware/controller_manager.h>
#include <pr2_controller_interface/controller.h>
#include <controller_manager/controller_loader.h>

namespace pr2_hardware
{

ControllerManager::ControllerManager(pr2_hardware_interface::HardwareInterface *hw, const ros::NodeHandle& nh) :
  model_(hw),
  state_(NULL),
  pr2_hardware_(NULL),
  nh_(nh)
{

}

ControllerManager::~ControllerManager()
{
  if (pr2_hardware_)
    delete pr2_hardware_;
}


bool ControllerManager::initXml(TiXmlElement* config)
{
  if (!model_.initXml(config))
  {
    ROS_ERROR("Failed to initialize pr2 mechanism model");
    return false;
  }
  pr2_hardware_ = new PR2Hardware(&model_);
  state_ = &pr2_hardware_->robot_state_;
  motors_previously_halted_ = state_->isHalted();

  cm_.reset(new controller_manager::ControllerManager(pr2_hardware_, nh_));

  typedef boost::shared_ptr<controller_manager::ControllerLoaderInterface> LoaderPtr;
  cm_->registerControllerLoader(LoaderPtr( new controller_manager::ControllerLoader<pr2_controller_interface::Controller>("pr2_controller_interface",
                                                                                                                          "pr2_controller_interface::Controller") ) );

  return true;
}


// Must be realtime safe.
void ControllerManager::update()
{
  state_->propagateActuatorPositionToJointPosition();
  state_->zeroCommands();

  // Restart all running controllers if motors are re-enabled
  bool reset_controllers = !state_->isHalted() && motors_previously_halted_;
  motors_previously_halted_ = state_->isHalted();

  // Update all controllers
  cm_->update(state_->getTime(), reset_controllers);

  state_->enforceSafety();
  state_->propagateJointEffortToActuatorEffort();
}

}
