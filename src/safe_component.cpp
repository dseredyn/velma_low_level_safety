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

#include <sstream>

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>
#include <kuka_lwr_fri/friComm.h>

#include "velma_low_level_safety.h"
#include <math.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <sys/time.h>

#include "common.h"

using namespace RTT;
using namespace velma_low_level_interface_msgs;

class VelmaLowSafeComponent: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit VelmaLowSafeComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

private:
    const int arm_joints_count_;

    void calculateArmDampingTorque(const Eigen::Matrix<double,7,1> &joint_velocity,
        const std::vector<double> &damping_factors, Eigen::Matrix<double,7,1> &joint_torque_command);

    void calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command);

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

    std::string cmdToStr(const VelmaLowLevelCommand &cmd);

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
    velma_lli_types::VelmaCommand_Ports<RTT::OutputPort > cmd_ports_out_;

    VelmaLowLevelStatus status_in_;
    velma_lli_types::VelmaStatus_Ports<RTT::InputPort > status_ports_in_;

    VelmaLowLevelCommand cmd_in_;
    RTT::InputPort<VelmaLowLevelCommand> port_command_in_;

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
};

VelmaLowSafeComponent::VelmaLowSafeComponent(const std::string &name) :
    TaskContext(name, PreOperational),
    arm_joints_count_(7),
    cmd_ports_out_(*this),
    status_ports_in_(*this),
    torso_damping_factor_(-1)  // initialize with invalid value, should be later set to >= 0
{
    this->ports()->addPort("command_INPORT", port_command_in_);
    this->ports()->addPort("status_test_OUTPORT", port_status_test_out_);

    this->ports()->addPort("status_rArm_friIntfState_INPORT", port_rArm_fri_state_in_);
    this->ports()->addPort("status_rArm_friRobotState_INPORT", port_rArm_robot_state_in_);

    this->ports()->addPort("status_lArm_friIntfState_INPORT", port_lArm_fri_state_in_);
    this->ports()->addPort("status_lArm_friRobotState_INPORT", port_lArm_robot_state_in_);

    this->ports()->addPort("cmd_rArm_cmd_OUTPORT", port_rArm_KRL_CMD_out_);
    this->ports()->addPort("cmd_lArm_cmd_OUTPORT", port_lArm_KRL_CMD_out_);

    addProperty("l_arm_damping_factors", l_arm_damping_factors_);
    addProperty("r_arm_damping_factors", r_arm_damping_factors_);
    addProperty("torso_damping_factor", torso_damping_factor_);

    addProperty("arm_q_limits_lo", arm_q_limits_lo_);
    addProperty("arm_q_limits_hi", arm_q_limits_hi_);
    addProperty("arm_dq_limits", arm_dq_limits_);
    addProperty("arm_t_limits", arm_t_limits_);
}

bool VelmaLowSafeComponent::configureHook() {
    Logger::In in("VelmaLowLevelSafety::configureHook");

    if (l_arm_damping_factors_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter l_arm_damping_factors is set to illegal value (wrong vector size: " <<
            l_arm_damping_factors_.size() << ")" << Logger::endl;
        return false;
    }

    if (r_arm_damping_factors_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter r_arm_damping_factors is set to illegal value (wrong vector size: " <<
            r_arm_damping_factors_.size() << ")" << Logger::endl;
        return false;
    }

    for (int i = 0; i < arm_joints_count_; ++i) {
        if (l_arm_damping_factors_[i] < 0) {
            Logger::log() << Logger::Error <<
                "parameter l_arm_damping_factors[" << i << "] is set to illegal value: " <<
                l_arm_damping_factors_[i] << Logger::endl;
            return false;
        }
        if (r_arm_damping_factors_[i] < 0) {
            Logger::log() << Logger::Error <<
                "parameter r_arm_damping_factors[" << i << "] is set to illegal value: " <<
                r_arm_damping_factors_[i] << Logger::endl;
            return false;
        }
    }

    if (torso_damping_factor_ < 0) {
        Logger::log() << Logger::Error <<
            "parameter torso_damping_factor is set to illegal value: " <<
            torso_damping_factor_ << Logger::endl;
        return false;
    }

    if (arm_q_limits_lo_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_q_limits_lo is set to illegal value (wrong vector size: " <<
            arm_q_limits_lo_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_q_limits_hi_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_q_limits_hi is set to illegal value (wrong vector size: " <<
            arm_q_limits_hi_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_dq_limits_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_dq_limits is set to illegal value (wrong vector size: " <<
            arm_dq_limits_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_t_limits_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_t_limits is set to illegal value (wrong vector size: " <<
            arm_t_limits_.size() << ")" << Logger::endl;
        return false;
    }

    return true;
}

bool VelmaLowSafeComponent::startHook() {
    return true;
}

void VelmaLowSafeComponent::stopHook() {
}

void VelmaLowSafeComponent::calculateArmDampingTorque(const Eigen::Matrix<double,7,1> &joint_velocity,
    const std::vector<double> &damping_factors, Eigen::Matrix<double,7,1> &joint_torque_command)
{
    joint_torque_command.setZero();
    for (int i = 0; i < arm_joints_count_; ++i) {
        joint_torque_command(i) = -damping_factors[i] * joint_velocity(i);
    }
}

void VelmaLowSafeComponent::calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command)
{
    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_constant = 0.00105;
    double joint_velocity = motor_velocity / torso_trans_mult;
    double motor_torque_command = -torso_damping_factor_ * joint_velocity;
    motor_current_command = motor_torque_command / torso_gear / torso_motor_constant;
}

void VelmaLowSafeComponent::updateHook() {
    //
    // read HW status
    //
    bool rArm_valid_prev = status_ports_in_.isValid("rArm");
    bool lArm_valid_prev = status_ports_in_.isValid("lArm");

    status_ports_in_.readPorts();
    status_ports_in_.convertToROS(status_in_);

    // as FRI components are not synchronized, their communication status should
    // be checked in two last cycles
    bool rArm_valid = rArm_valid_prev || status_ports_in_.isValid("rArm");
    bool lArm_valid = lArm_valid_prev || status_ports_in_.isValid("lArm");

    // check FRI and LWR state
    // as FRI components may not be synchronized
    bool rHand_valid = true;
    bool lHand_valid = true;
    bool tMotor_valid = true;
    bool hpMotor_valid = true;
    bool htMotor_valid = true;

/*
// TODO
    if (port_rArm_fri_state_in_.read(rArm_fri_state_) == RTT::NewData && port_rArm_robot_state_in_.read(rArm_robot_state_) == RTT::NewData) {
        if ( !isLwrOk(rArm_fri_state_, rArm_robot_state_) ) {
            rArm_valid = false;
        }
    }
    if (port_lArm_fri_state_in_.read(lArm_fri_state_) == RTT::NewData && port_lArm_robot_state_in_.read(lArm_robot_state_) == RTT::NewData) {
        if ( !isLwrOk(lArm_fri_state_, lArm_robot_state_) ) {
            lArm_valid = false;
        }
    }

    if ( !isStatusValid(status_in_.rArm, id_faulty_submodule) ) {
        rArm_valid = false;
    }
    if ( !isStatusValid(status_in_.lArm, id_faulty_submodule) ) {
        lArm_valid = false;
    }

    if ( !status_ports_in_.isValid("rHand")) {
        rHand_valid = false;
    }

    if ( !isStatusValid(status_in_.rHand, id_faulty_submodule) ) {
        rHand_valid = false;
    }

    if ( !status_ports_in_.isValid("lHand")) {
        lHand_valid = false;
    }

    if ( !isStatusValid(status_in_.lHand, id_faulty_submodule) ) {
        lHand_valid = false;
    }

    if ( !status_ports_in_.isValid("tMotor")) {
        tMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.tMotor, id_faulty_submodule) ) {
        tMotor_valid = false;
    }

    if ( !status_ports_in_.isValid("hpMotor")) {
        hpMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.hpMotor, id_faulty_submodule) ) {
        hpMotor_valid = false;
    }

    if ( !status_ports_in_.isValid("htMotor")) {
        htMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.htMotor, id_faulty_submodule) ) {
        htMotor_valid = false;
    }
*/
    allHwOk_ =  rArm_valid      && lArm_valid &&
                rHand_valid     && lHand_valid &&
                tMotor_valid    && hpMotor_valid &&
                htMotor_valid;
/*
    //
    // read commands
    //
    readCmdData_ = (port_command_in_.read(cmd_in_) == NewData);
    cmdValid_ = false;
    if (!readCmdData_) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_UP, 0, 0);
    }
    else if (!isCommandValid(cmd_in_, id_faulty_module, id_faulty_submodule)) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_COMMAND, id_faulty_module, id_faulty_submodule);
    }
    else if (VelmaLowLevelStatusSC::STATE_HW_DOWN != state_ && cmd_in_.test != packet_counter_) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_PACKET_LOST, 0, 0);
    }
    else {
        cmdValid_ = true;
    }
*/

    //
    // write HW commands to available devices
    //

    // generate safe outputs for all operational devices
    if (rArm_valid) {
        cmd_ports_out_.setValid("rArm", true);
        arm_dq_.convertFromROS(status_in_.rArm.dq);
        calculateArmDampingTorque(arm_dq_.data_, r_arm_damping_factors_, arm_t_cmd_.data_);
        arm_t_cmd_.convertToROS(cmd_out_.rArm.t);
    }

    if (lArm_valid) {
        cmd_ports_out_.setValid("lArm", true);
        arm_dq_.convertFromROS(status_in_.lArm.dq);
        calculateArmDampingTorque(arm_dq_.data_, l_arm_damping_factors_, arm_t_cmd_.data_);
        arm_t_cmd_.convertToROS(cmd_out_.lArm.t);
    }

    if (tMotor_valid) {
        cmd_ports_out_.setValid("tMotor", true);
        calculateTorsoDampingTorque(status_in_.tMotor.dq, cmd_out_.tMotor.i);
        cmd_out_.tMotor.q = 0;
        cmd_out_.tMotor.dq = 0;
    }

    if (hpMotor_valid) {
        cmd_ports_out_.setValid("hpMotor", true);
        cmd_out_.hpMotor.i = 0;
        cmd_out_.hpMotor.q = status_in_.hpMotor.q;
        cmd_out_.hpMotor.dq = 0;
    }

    if (htMotor_valid) {
        cmd_ports_out_.setValid("htMotor", true);
        cmd_out_.htMotor.i = 0;
        cmd_out_.htMotor.q = status_in_.htMotor.q;
        cmd_out_.htMotor.dq = 0;
    }

    //
    // set HW commands
    //
    cmd_ports_out_.convertFromROS(cmd_out_);
/*
    // TODO
        if (VelmaLowLevelStatusSC::STATE_HW_ENABLED == state_ && cmd_in_.sc.valid && cmd_in_.sc.cmd == 1) {
            //
            // write FRI commands
            //
            if (rArm_fri_state_.state == FRI_STATE_MON) {
                rArm_KRL_CMD_.data = 1;
                port_rArm_KRL_CMD_out_.write(rArm_KRL_CMD_);
            }

            if (lArm_fri_state_.state == FRI_STATE_MON) {
                lArm_KRL_CMD_.data = 1;
                port_lArm_KRL_CMD_out_.write(lArm_KRL_CMD_);
            }
        }
*/
    //
    // write test field
    //
    ++packet_counter_;
    status_test_out_ = packet_counter_;
    port_status_test_out_.write(status_test_out_);

    //
    // write commands
    //
    cmd_ports_out_.writePorts();

//    UNRESTRICT_ALLOC;
}

ORO_LIST_COMPONENT_TYPE(VelmaLowSafeComponent)

