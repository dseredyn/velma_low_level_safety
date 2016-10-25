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

#ifndef VELMA_LOW_LEVEL_SAFETY_H_
#define VELMA_LOW_LEVEL_SAFETY_H_

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <std_msgs/UInt32.h>

#include "velma_low_level_interface_msgs/VelmaLowLevelCommand.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatus.h"

#include "eigen_conversions/eigen_msg.h"

#include "velma_low_level_interface/velma_lli_command_ports.h"
#include "velma_low_level_interface/velma_lli_status_ports.h"

#include <sys/time.h>

using namespace velma_low_level_interface_msgs;

class VelmaLowLevelSafety: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit VelmaLowLevelSafety(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

private:
    const int arm_joints_count_;

    void calculateArmDampingTorque(const Eigen::Matrix<double,7,1> &joint_velocity,
        const std::vector<double> &damping_factors, Eigen::Matrix<double,7,1> &joint_torque_command);

    void calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command);

    bool isCommandValidTorso(const VelmaLowLevelCommandMotor &cmd) const;
    bool isCommandValidHeadPan(const VelmaLowLevelCommandMotor &cmd) const;
    bool isCommandValidHeadTilt(const VelmaLowLevelCommandMotor &cmd) const;
    bool isCommandValid(const VelmaLowLevelCommandArm &cmd) const;
    bool isCommandValid(const VelmaLowLevelCommandHand &cmd) const;
    bool isCommandValid(const VelmaLowLevelCommand &cmd) const;

    bool isStatusValid(const VelmaLowLevelStatus &st) const;
    bool isStatusValid(const VelmaLowLevelStatusArm &st) const;
    bool isStatusValid(const VelmaLowLevelStatusHand &st) const;
    bool isStatusValid(const VelmaLowLevelStatusMotor &st) const;
    bool isStatusValid(const VelmaLowLevelStatusFT &st) const;

    bool isLwrOk(const tFriIntfState &fri_state, const tFriRobotState &robot_state) const;

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

//    enum SafetyControllerState {HW_DOWN, HW_DISABLED, HW_ENABLED, CONTROL_ENABLED};
    static const std::string state_names_[5];
//    const std::string& getStateName(SafetyControllerState state) const;
    const std::string& getStateName(int32_t state) const;

    std::string cmdToStr(const VelmaLowLevelCommand &cmd);

//    SafetyControllerState state_;
    int32_t state_;
    int counts_HW_DISABLED_;

/*
    states: {HW_DOWN, HW_DISABLED, HW_ENABLED, CONTROL_ENABLED};

    possible state changes:
    HW_DOWN -> HW_DISABLED
    HW_DISABLED -> HW_ENABLED       (can be changed explicitily)
    HW_ENABLED -> CONTROL_ENABLED   (can be changed explicitily)
    CONTROL_ENABLED -> HW_ENABLED
    CONTROL_ENABLED -> HW_DISABLED
    CONTROL_ENABLED -> HW_DOWN
    HW_ENABLED -> HW_DISABLED
    HW_ENABLED -> HW_DOWN
    HW_DISABLED -> HW_DOWN

    initial conditions for:

    HW_ENABLED (enable all HW devices):
    - the state id HW_DISABLED
    - there is communication with all HW devices
    - all status data is valid
    - a proper safety controller command is sent (ENABLE_HW)


    CONTROL_ENABLED (turn off the safety controller):
    - the state is HW_ENABLED
    - there is communication with all HW devices
    - all HW devices are enabled
    - all status data is valid
    - all control data is valid
    - a proper safety controller command is sent (ENABLE_CONTROL)
*/
    // ROS parameters
	std::vector<double> l_arm_damping_factors_;
	std::vector<double> r_arm_damping_factors_;
	double torso_damping_factor_;

    std::vector<double> arm_q_limits_lo_;
    std::vector<double> arm_q_limits_hi_;
    std::vector<double> arm_dq_limits_;
    std::vector<double> arm_t_limits_;

    // port data

    VelmaLowLevelCommand cmd_out_;
    VelmaLLICommandOutput cmd_ports_out_;

    VelmaLowLevelStatus status_in_;
    VelmaLLIStatusInput status_ports_in_;

    VelmaLowLevelCommand cmd_in_;
    RTT::InputPort<VelmaLowLevelCommand> port_command_in_;

    VelmaLowLevelStatusSC status_sc_out_;
    RTT::OutputPort<VelmaLowLevelStatusSC> port_status_sc_out_;

    uint32_t status_test_out_;
    RTT::OutputPort<uint32_t> port_status_test_out_;

    // additional HW control ports
    RTT::InputPort<tFriIntfState>       port_rArm_fri_state_in_;
    RTT::InputPort<tFriRobotState>      port_rArm_robot_state_in_;
    RTT::InputPort<tFriIntfState>       port_lArm_fri_state_in_;
    RTT::InputPort<tFriRobotState>      port_lArm_robot_state_in_;
    RTT::OutputPort<std_msgs::Int32 >   port_rArm_KRL_CMD_out_;             // FRIx.KRL_CMD
    RTT::OutputPort<std_msgs::Int32 >   port_lArm_KRL_CMD_out_;             // FRIx.KRL_CMD

    // additional status port
    RTT::OutputPort<std_msgs::UInt32 >   port_robot_status_out_;

    tFriIntfState       rArm_fri_state_;
    tFriRobotState      rArm_robot_state_;
    tFriIntfState       lArm_fri_state_;
    tFriRobotState      lArm_robot_state_;
    std_msgs::Int32     rArm_KRL_CMD_;             // FRIx.KRL_CMD
    std_msgs::Int32     lArm_KRL_CMD_;             // FRIx.KRL_CMD

    velma_lli_types::PortRawData<Eigen::Matrix<double,7,1>, boost::array<double, 7ul> > arm_dq_;
    velma_lli_types::PortRawData<Eigen::Matrix<double,7,1>, boost::array<double, 7ul> > arm_t_cmd_;

    bool emergency_;

    int no_hw_error_counter_;

    bool allHwOk_;
    bool hwStatusValid_;
    bool readCmdData_;
    bool cmdValid_;


    uint32_t packet_counter_;
    ros::Time wall_time_prev_;

    struct timeval time_prev_;
};

#endif  // VELMA_LOW_LEVEL_SAFETY_H_

