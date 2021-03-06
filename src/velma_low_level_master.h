/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef VELMA_LOW_LEVEL_MASTER_H_
#define VELMA_LOW_LEVEL_MASTER_H_

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"

#include <std_msgs/UInt32.h>

#include "velma_low_level_interface_msgs/VelmaLowLevelCommand.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatus.h"

#include "eigen_conversions/eigen_msg.h"

#include "velma_low_level_interface/velma_lli_command_ports.h"
#include "velma_low_level_interface/velma_lli_status_ports.h"

#include "common_behavior/abstract_behavior.h"
#include "common_behavior/abstract_state.h"

#include <sys/time.h>

using namespace velma_low_level_interface_msgs;

class VelmaLowLevelMaster: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit VelmaLowLevelMaster(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

private:

    // parameters
    std::vector<std::string > state_names_;
    std::string initial_state_name_;

    std::vector<std::string > behavior_names_;

    // port data
    VelmaLowLevelCommand cmd_out_;
    velma_lli_types::VelmaCommand_Ports<RTT::OutputPort > cmd_ports_out_;

    VelmaLowLevelStatus status_in_;
    velma_lli_types::VelmaStatus_Ports<RTT::InputPort > status_ports_in_;

    VelmaLowLevelCommand cmd_in_;
    RTT::InputPort<VelmaLowLevelCommand> port_command_in_;

    VelmaLowLevelStatusSC status_sc_out_;
    RTT::OutputPort<VelmaLowLevelStatusSC> port_status_sc_out_;

    uint32_t status_test_out_;
    RTT::OutputPort<uint32_t> port_status_test_out_;

    std::vector<std::shared_ptr<BehaviorBase<VelmaLowLevelStatus, VelmaLowLevelCommand> > > behaviors_;

    std::vector<std::shared_ptr<StateBase<VelmaLowLevelStatus, VelmaLowLevelCommand> > > states_;
    std::shared_ptr<StateBase<VelmaLowLevelStatus, VelmaLowLevelCommand> > current_state_;

    // pointer to conman scheme TaskContext
    TaskContext *scheme_;

    // conman scheme operations
    RTT::OperationCaller<bool(const std::string &)> hasBlock_;
    RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)> addGraphConfiguration_;
    RTT::OperationCaller<bool(int)> switchToConfiguration_;

    std::vector<TaskContext* > scheme_peers_;
    std::vector<std::vector<bool> > is_running_;

/*
    // additional HW control ports
    RTT::InputPort<tFriIntfState>       port_rArm_fri_state_in_;
    RTT::InputPort<tFriRobotState>      port_rArm_robot_state_in_;
    RTT::InputPort<tFriIntfState>       port_lArm_fri_state_in_;
    RTT::InputPort<tFriRobotState>      port_lArm_robot_state_in_;
    RTT::OutputPort<std_msgs::Int32 >   port_rArm_KRL_CMD_out_;             // FRIx.KRL_CMD
    RTT::OutputPort<std_msgs::Int32 >   port_lArm_KRL_CMD_out_;             // FRIx.KRL_CMD

    // additional status port
//    RTT::OutputPort<std_msgs::UInt32 >   port_robot_status_out_;

    tFriIntfState       rArm_fri_state_;
    tFriRobotState      rArm_robot_state_;
    tFriIntfState       lArm_fri_state_;
    tFriRobotState      lArm_robot_state_;
    std_msgs::Int32     rArm_KRL_CMD_;             // FRIx.KRL_CMD
    std_msgs::Int32     lArm_KRL_CMD_;             // FRIx.KRL_CMD

    interface_ports::PortRawData<Eigen::Matrix<double,7,1>, boost::array<double, 7ul> > arm_dq_;
    interface_ports::PortRawData<Eigen::Matrix<double,7,1>, boost::array<double, 7ul> > arm_t_cmd_;

    bool emergency_;

    int no_hw_error_counter_;

    bool allHwOk_;
    bool hwStatusValid_;
    bool readCmdData_;
    bool cmdValid_;


    uint32_t packet_counter_;
    ros::Time wall_time_prev_;

    struct timeval time_prev_;
*/
};

#endif  // VELMA_LOW_LEVEL_MASTER_H_

