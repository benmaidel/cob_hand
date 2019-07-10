/*
 * Copyright 2017 Mojin Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#include <cob_hand_bridge/InitFinger.h>
#include <cob_hand_bridge/Status.h>

#include <std_srvs/Trigger.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <diagnostic_updater/publisher.h>

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <angles/angles.h>

#include <gpio.h>
#include <sdhx.h>

boost::mutex g_mutex;

boost::shared_ptr<cob_hand_bridge::Status> g_status;

sensor_msgs::JointState g_js;

cob_hand_bridge::JointValues g_command;
cob_hand_bridge::JointValues g_default_command;
double g_stopped_velocity;
double g_stopped_current;
bool g_initialized;
bool g_motion_stopped;
bool g_control_stopped;
bool g_motors_moved;
std::vector<double> g_goal_tolerance;
std::string g_port;

// SDHX
boost::scoped_ptr<SDHX> g_sdhx;

bool isFingerReady_nolock() {
    return g_status && (g_status->status & (g_status->MASK_FINGER_READY | g_status->MASK_ERROR)) == g_status->MASK_FINGER_READY && g_status->rc == 0;
}


bool halt(){
    if(!g_sdhx || !g_sdhx->halt()){
        ROS_ERROR("Halt service did not succeed");
        return false;
    }
    else
        return true;
}

bool init(){
    if(!g_sdhx){
        g_sdhx.reset(new SDHX);
        g_port = "/dev/ttyACM0";
        if(g_sdhx->init(g_port.c_str(), 0, 0, 0, 0) ){

            g_status->rc = 0;
            g_status->status &= ~g_status->MASK_ERROR;
            return true;
        }
    }
    return false;
}

bool recover(){
    if(g_sdhx) {
        if((g_status->status & g_status->MASK_ERROR) == 0 && g_status->rc == 0) {
            return true;
        }else if(g_sdhx->isInitialized()) {
            g_sdhx.reset();
            g_status->rc = 0;
            if(init()){
                g_status->status &= ~g_status->MASK_ERROR;
                return true;
            }
        }else{
            return false;
        }
    }
    return false;
}

void move(double j1, double j2){

    if(g_sdhx){
        int16_t position_cdeg[2];
        int16_t velocity_cdeg_s[2];
        int16_t current_100uA[2];
        position_cdeg[0] = angles::to_degrees(j1)*100;     position_cdeg[1]=angles::to_degrees(j2)*100;
        velocity_cdeg_s[0] = 0; velocity_cdeg_s[1] = 0;
        current_100uA[0] = 100;     current_100uA[1] = 100;
        g_sdhx->move(position_cdeg, velocity_cdeg_s, current_100uA);
    }
}

int main(int argc, char* argv[])
{
    
    g_stopped_velocity = 0.05;

    double stopped_current = 0.1;

    g_stopped_current = stopped_current * 1000.0; // (A -> 100uA)
    
    g_default_command.current_100uA[0] = 2120;
    g_default_command.current_100uA[1] = 1400;

    g_default_command.velocity_cdeg_s[0] = 1000;
    g_default_command.velocity_cdeg_s[1] = 1000;

    g_js.position.resize(g_command.position_cdeg.size());
    g_js.velocity.resize(g_command.position_cdeg.size());

    
    while(true){
      std::cout<<"1: init"<<std::endl;
      std::cout<<"2: recover"<<std::endl;
      std::cout<<"3: halt"<<std::endl;
      std::cout<<"4: move home"<<std::endl;
      std::cout<<"5: move close"<<std::endl;
      std::cout<<"6: move open"<<std::endl;
      int value;
      std::cin>>value;

      switch(value)
      {
      case 1:
        std::cout<<"init was "<<init()<<std::endl;
        break;
      case 2:
        std::cout<<"recover was "<<recover()<<std::endl;
        break;
      case 3:
        std::cout<<"halt was "<<halt()<<std::endl;
        break;
      case 4:
        move(0.6, 0.4);
        break;
      case 5:
        move(0.85, -0.1);
        break;
      case 6:
        move(-0.85, 1.4);
        break;
      }
    }
    return 0;
}
