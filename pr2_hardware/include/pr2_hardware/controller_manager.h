///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008-2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
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

#ifndef PR2_HARDWARE_CONTROLLER_MANAGER_H
#define PR2_HARDWARE_CONTROLLER_MANAGER_H

#include <tinyxml.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/robot.h>
#include <controller_manager/controller_manager.h>
#include <pr2_hardware/pr2_hardware.h>
#include <boost/scoped_ptr.hpp>

namespace pr2_hardware
{

class ControllerManager
{

public:
  ControllerManager(pr2_hardware_interface::HardwareInterface *hw,
                    const ros::NodeHandle& nh=ros::NodeHandle());
  ~ControllerManager();

  // Real-time functions
  void update();

  // Non real-time functions
  bool initXml(TiXmlElement* config);

  pr2_mechanism_model::Robot model_;
  PR2Hardware *state_;

private:
  bool motors_previously_halted_;
  boost::scoped_ptr<controller_manager::ControllerManager> cm_;
  ros::NodeHandle nh_;
};

}

#endif
