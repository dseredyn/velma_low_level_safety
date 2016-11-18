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

void setFault(VelmaLowLevelStatusSC &status, int type, int faulty_module_id, int faulty_submodule_id) {
    if ( !status.error ) {
        status.error = true;
        status.fault_type = type;
        status.faulty_module_id = faulty_module_id;
        status.faulty_submodule_id = faulty_submodule_id;
    }
}

VelmaLowLevelSafety::VelmaLowLevelSafety(const std::string &name) :
    TaskContext(name, PreOperational),
    arm_joints_count_(7),
    state_(VelmaLowLevelStatusSC::STATE_HW_DOWN),
    cmd_ports_out_(*this),
    status_ports_in_(*this),
    torso_damping_factor_(-1)  // initialize with invalid value, should be later set to >= 0
{
    this->ports()->addPort("command_INPORT", port_command_in_);
    this->ports()->addPort("status_sc_OUTPORT", port_status_sc_out_);
    this->ports()->addPort("status_test_OUTPORT", port_status_test_out_);

    this->ports()->addPort("status_rArm_friIntfState_INPORT", port_rArm_fri_state_in_);
    this->ports()->addPort("status_rArm_friRobotState_INPORT", port_rArm_robot_state_in_);

    this->ports()->addPort("status_lArm_friIntfState_INPORT", port_lArm_fri_state_in_);
    this->ports()->addPort("status_lArm_friRobotState_INPORT", port_lArm_robot_state_in_);

    this->ports()->addPort("cmd_rArm_cmd_OUTPORT", port_rArm_KRL_CMD_out_);
    this->ports()->addPort("cmd_lArm_cmd_OUTPORT", port_lArm_KRL_CMD_out_);

//    this->ports()->addPort("port_robot_status_OUTPORT", port_robot_status_out_);

    addProperty("l_arm_damping_factors", l_arm_damping_factors_);
    addProperty("r_arm_damping_factors", r_arm_damping_factors_);
    addProperty("torso_damping_factor", torso_damping_factor_);

    addProperty("arm_q_limits_lo", arm_q_limits_lo_);
    addProperty("arm_q_limits_hi", arm_q_limits_hi_);
    addProperty("arm_dq_limits", arm_dq_limits_);
    addProperty("arm_t_limits", arm_t_limits_);
}

bool VelmaLowLevelSafety::configureHook() {
    Logger::In in("VelmaLowLevelSafety::configureHook");

    double x_nan = NAN;

    if (!isNaN(x_nan)) {
        Logger::log() << Logger::Error << "NaN testing is not supported (NaN == NaN)" << Logger::endl;
        return false;
    }

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

bool VelmaLowLevelSafety::startHook() {
//    RESTRICT_ALLOC;

    emergency_ = false;

    no_hw_error_counter_ = 0;

    state_ = VelmaLowLevelStatusSC::STATE_HW_DOWN;
    allHwOk_ = false;
    hwStatusValid_ = false;
    readCmdData_ = false;
    cmdValid_ = false;

    packet_counter_ = 1;

    wall_time_prev_ = rtt_rosclock::rtt_wall_now();
    gettimeofday(&time_prev_, NULL);

//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaLowLevelSafety::stopHook() {
}

void VelmaLowLevelSafety::calculateArmDampingTorque(const Eigen::Matrix<double,7,1> &joint_velocity,
    const std::vector<double> &damping_factors, Eigen::Matrix<double,7,1> &joint_torque_command)
{
    joint_torque_command.setZero();
    for (int i = 0; i < arm_joints_count_; ++i) {
        joint_torque_command(i) = -damping_factors[i] * joint_velocity(i);
    }
}

void VelmaLowLevelSafety::calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command)
{
    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_constant = 0.00105;
    double joint_velocity = motor_velocity / torso_trans_mult;
    double motor_torque_command = -torso_damping_factor_ * joint_velocity;
    motor_current_command = motor_torque_command / torso_gear / torso_motor_constant;
}

std::string VelmaLowLevelSafety::cmdToStr(const VelmaLowLevelCommand &cmd) {
    // TODO: this is surely non-RT

    std::ostringstream strs;
    strs << cmd.tMotor.i << " " << cmd.hpMotor.i << " " << cmd.htMotor.i << " " << cmd.hpMotor.q << " " << cmd.htMotor.q << " " <<
        cmd.hpMotor.dq << " " << cmd.htMotor.dq << " " << cmd.lArm.t.size() << " " << cmd.rArm.t.size() << " ";

    for (int i = 0; i < arm_joints_count_; ++i) {
        strs << cmd.lArm.t[i] << " ";
        strs << cmd.rArm.t[i] << " ";
    }

    strs << cmd.lHand.q.size() << " " << cmd.lHand.dq.size() << " " << cmd.lHand.max_p.size() << " " <<
        cmd.lHand.max_i.size() << " " << cmd.rHand.q.size() << " " << cmd.rHand.dq.size() << " " <<
        cmd.rHand.max_p.size() << " " << cmd.rHand.max_i.size() << " ";

    for (int i = 0; i < 4; ++i) {
        strs << cmd.lHand.q[i] << " ";
        strs << cmd.lHand.dq[i] << " ";
        strs << cmd.lHand.max_p[i] << " ";
        strs << cmd.lHand.max_i[i] << " ";
        strs << cmd.rHand.q[i] << " ";
        strs << cmd.rHand.dq[i] << " ";
        strs << cmd.rHand.max_p[i] << " ";
        strs << cmd.rHand.max_i[i] << " ";
    }

    std::string str = strs.str();
    return str;
}

void VelmaLowLevelSafety::updateHook() {
//    RESTRICT_ALLOC;

    // reset error status
    status_sc_out_.error = false;
    status_sc_out_.fault_type = 0;
    status_sc_out_.faulty_module_id = 0;
    status_sc_out_.faulty_submodule_id = 0;

    int id_faulty_module;
    int id_faulty_submodule;

    //
    // read HW status
    //
    bool rArm_valid_prev = status_ports_in_.isValid("rArm");
    bool lArm_valid_prev = status_ports_in_.isValid("lArm");

    status_ports_in_.readPorts();
    status_ports_in_.convertToROS(status_in_);

    bool allHwOk_prev = allHwOk_;
    bool readCmdData_prev = readCmdData_;
    bool cmdValid_prev = cmdValid_;

    // as FRI components are not synchronized, their communication status should
    // be checked in two last cycles
    bool rArm_valid = rArm_valid_prev || status_ports_in_.isValid("rArm");
    bool lArm_valid = lArm_valid_prev || status_ports_in_.isValid("lArm");

    if (!rArm_valid) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_R_ARM, 0);
    }

    if (!lArm_valid) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_L_ARM, 0);
    }

    // check FRI and LWR state
    // as FRI components may not be synchronized
    if (port_rArm_fri_state_in_.read(rArm_fri_state_) == RTT::NewData && port_rArm_robot_state_in_.read(rArm_robot_state_) == RTT::NewData) {
        if ( !isLwrOk(rArm_fri_state_, rArm_robot_state_) ) {
            setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_HW_STATE, VelmaLowLevelStatusSC::MODULE_R_ARM, 0);
            rArm_valid = false;
        }
    }
    if (port_lArm_fri_state_in_.read(lArm_fri_state_) == RTT::NewData && port_lArm_robot_state_in_.read(lArm_robot_state_) == RTT::NewData) {
        if ( !isLwrOk(lArm_fri_state_, lArm_robot_state_) ) {
            setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_HW_STATE, VelmaLowLevelStatusSC::MODULE_L_ARM, 0);
            lArm_valid = false;
        }
    }

    if ( !isStatusValid(status_in_.rArm, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_R_ARM, id_faulty_submodule);
        rArm_valid = false;
    }
    if ( !isStatusValid(status_in_.lArm, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_L_ARM, id_faulty_submodule);
        lArm_valid = false;
    }

    bool rHand_valid = true;
    bool lHand_valid = true;
    bool rFt_valid = true;
    bool lFt_valid = true;
    bool tMotor_valid = true;
    bool hpMotor_valid = true;
    bool htMotor_valid = true;

    if ( !status_ports_in_.isValid("rHand")) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_R_HAND, 0);
        rHand_valid = false;
    }

    if ( !isStatusValid(status_in_.rHand, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_R_HAND, id_faulty_submodule);
        rHand_valid = false;
    }

    if ( !status_ports_in_.isValid("lHand")) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_L_HAND, 0);
        lHand_valid = false;
    }

    if ( !isStatusValid(status_in_.lHand, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_L_HAND, id_faulty_submodule);
        lHand_valid = false;
    }

    if ( !status_ports_in_.isValid("rFt")) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_R_FT, 0);
        rFt_valid = false;
    }

    if ( !isStatusValid(status_in_.rFt, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_R_FT, id_faulty_submodule);
        rFt_valid = false;
    }

    if ( !status_ports_in_.isValid("lFt")) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_L_FT, 0);
        lFt_valid = false;
    }

    if ( !isStatusValid(status_in_.lFt, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_L_FT, id_faulty_submodule);
        lFt_valid = false;
    }

    if ( !status_ports_in_.isValid("tMotor")) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_T_MOTOR, 0);
        tMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.tMotor, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_T_MOTOR, id_faulty_submodule);
        tMotor_valid = false;
    }

    if ( !status_ports_in_.isValid("hpMotor")) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_HP_MOTOR, 0);
        hpMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.hpMotor, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_HP_MOTOR, id_faulty_submodule);
        hpMotor_valid = false;
    }

    if ( !status_ports_in_.isValid("htMotor")) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_COMM_HW, VelmaLowLevelStatusSC::MODULE_HT_MOTOR, 0);
        htMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.htMotor, id_faulty_submodule) ) {
        setFault(status_sc_out_, VelmaLowLevelStatusSC::FAULT_NAN_STATUS, VelmaLowLevelStatusSC::MODULE_HT_MOTOR, id_faulty_submodule);
        htMotor_valid = false;
    }

    allHwOk_ =  rArm_valid      && lArm_valid &&
                rHand_valid     && lHand_valid &&
                rFt_valid       && lFt_valid &&
                tMotor_valid    && hpMotor_valid &&
                htMotor_valid;

    // send diagnostic information about robot state
//    std_msgs::UInt32 robot_state;
//    robot_state.data =  rArm_valid          | (lArm_valid<<1) |
//                        (rHand_valid<<2)    | (lHand_valid<<3) |
//                        (rFt_valid<<4)      | (lFt_valid<<5) |
//                        (tMotor_valid<<6)   | (hpMotor_valid<<7) |
//                        (htMotor_valid<<8);

//    port_robot_status_out_.write(robot_state);

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

//    if (readCmdData_ && cmd_in_.test != packet_counter_ && VelmaLowLevelStatusSC::STATE_HW_DOWN != state_) {
//        Logger::In in("VelmaLowLevelSafety::updateHook");
//        Logger::log() << Logger::Warning << "received wrong cmd frame: " << cmd_in_.test << " should be " << packet_counter_ << Logger::endl;
//    }

    const int32_t prev_state = state_;

    //
    // manage FSM state transitions
    //
    if (VelmaLowLevelStatusSC::STATE_HW_DOWN == state_) {
        if (allHwOk_) {
            state_ = VelmaLowLevelStatusSC::STATE_HW_DISABLED;
        }
    }
    else if (VelmaLowLevelStatusSC::STATE_HW_DISABLED == state_) {
        // state changes
        if (!allHwOk_) {
            // one or more HW components are down
            state_ = VelmaLowLevelStatusSC::STATE_HW_DOWN;
        }
        else if (cmd_in_.sc.valid && cmd_in_.sc.cmd == 1) {
            state_ = VelmaLowLevelStatusSC::STATE_HW_ENABLED;
            //Logger::log() << Logger::Info << "accepted cmd: enable_hw" << Logger::endl;
        }
    }
    else if (VelmaLowLevelStatusSC::STATE_HW_ENABLED == state_) {
        if (!allHwOk_) {
            state_ = VelmaLowLevelStatusSC::STATE_HW_DOWN;
        }
        else if (cmdValid_ && cmd_in_.sc.valid && cmd_in_.sc.cmd == 2) {
            // change state to STATE_HW_ENABLED
            state_ = VelmaLowLevelStatusSC::STATE_CONTROL_ENABLED;
            //Logger::log() << Logger::Info << "accepted cmd: enable_control" << Logger::endl;
        }
    }
    else if (VelmaLowLevelStatusSC::STATE_CONTROL_ENABLED == state_) {
        if (!allHwOk_) {
            state_ = VelmaLowLevelStatusSC::STATE_HW_DOWN;
        }
        else if (!cmdValid_) {
            state_ = VelmaLowLevelStatusSC::STATE_HW_ENABLED;
        }
    }

    if (prev_state != state_) {
//        Logger::log() << Logger::Info << "state change: " << getStateName(prev_state) << " -> " << getStateName(state_) << Logger::endl;
    }

    //
    // execute states behaviours
    //
    if (VelmaLowLevelStatusSC::STATE_HW_DOWN == state_) {
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
    }
    else if (VelmaLowLevelStatusSC::STATE_HW_DISABLED == state_ || VelmaLowLevelStatusSC::STATE_HW_ENABLED == state_) {
        arm_dq_.convertFromROS(status_in_.rArm.dq);
        calculateArmDampingTorque(arm_dq_.data_, r_arm_damping_factors_, arm_t_cmd_.data_);
        arm_t_cmd_.convertToROS(cmd_out_.rArm.t);

        arm_dq_.convertFromROS(status_in_.lArm.dq);
        calculateArmDampingTorque(arm_dq_.data_, l_arm_damping_factors_, arm_t_cmd_.data_);
        arm_t_cmd_.convertToROS(cmd_out_.lArm.t);

        calculateTorsoDampingTorque(status_in_.tMotor.dq, cmd_out_.tMotor.i);

        cmd_out_.hpMotor.i = 0;
        cmd_out_.hpMotor.q = status_in_.hpMotor.q;
        cmd_out_.hpMotor.dq = 0;

        cmd_out_.htMotor.i = 0;
        cmd_out_.htMotor.q = status_in_.htMotor.q;
        cmd_out_.htMotor.dq = 0;

        cmd_out_.rTact.cmd = 0;
        cmd_out_.rTact.valid = false;
        cmd_out_.rHand.valid = false;
        cmd_out_.lHand.valid = false;

        //
        // set HW commands
        //
        cmd_ports_out_.convertFromROS(cmd_out_);
        cmd_ports_out_.setValid(true);

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
    }
    else if (VelmaLowLevelStatusSC::STATE_CONTROL_ENABLED == state_) {
        //
        // set HW commands
        //
        cmd_ports_out_.convertFromROS(cmd_in_);
        cmd_ports_out_.setValid(true);
    }

    //
    // write test field
    //
    ++packet_counter_;
    status_test_out_ = packet_counter_;
    port_status_test_out_.write(status_test_out_);

    //
    // write status
    //
    status_sc_out_.state_id = static_cast<int32_t >(state_);
    port_status_sc_out_.write(status_sc_out_);

    //
    // write commands
    //
    cmd_ports_out_.writePorts();

//    UNRESTRICT_ALLOC;
}

ORO_LIST_COMPONENT_TYPE(VelmaLowLevelSafety)
ORO_CREATE_COMPONENT_LIBRARY();

