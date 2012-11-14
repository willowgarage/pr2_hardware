///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
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

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <pr2_hardware/pr2_hardware.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DummyPR2App");

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  pr2_hardware_interface::HardwareInterface hw;
  pr2_mechanism_model::Robot* cm2_model = new pr2_mechanism_model::Robot(&hw);

  std::string urdf_string;
  bool success;
  success = nh.getParam("/robot_description", urdf_string);
  if (!success)
  {
    printf("Couldn't find urdf\n");
    return -1;
  }

  TiXmlDocument doc;
  success = doc.Parse(urdf_string.c_str());
  if (!success)
  {
    printf("Couldn't parse urdf\n");
    return -1;
  }

  if (doc.Error())
  {
    printf("Error during urdf parsing\n");
  }

  printf("About to call initXml\n");
  success = cm2_model->initXml(doc.RootElement());
  if (!success)
  {
    printf("Error calling initXml\n");
    return -1;
  }
  printf("Done calling initXml\n");

  pr2_hardware::PR2Hardware* cm2_state = new pr2_hardware::PR2Hardware(cm2_model);

  controller_manager::ControllerManager* cm = new controller_manager::ControllerManager(cm2_state, nh);

  while (ros::ok())
  {
    ROS_INFO("loop");
    //hw.read();
    cm->update(ros::Time::now());
    //hw.write();

    ROS_INFO("about to sleep\n");
    break;
    ros::Duration(1.0).sleep();
  }

  delete cm;
  delete cm2_state;
  delete cm2_model;


  return 0;
}

