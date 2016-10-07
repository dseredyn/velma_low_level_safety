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

using namespace RTT;

const std::string VelmaLowLevelSafety::state_names_[5] = {"HW_DOWN", "HW_DISABLED", "HW_ENABLED", "CONTROL_ENABLED", "unknown"};

const std::string& VelmaLowLevelSafety::getStateName(int32_t state) const {
    if (state == VelmaLowLevelStatusSC::HW_DOWN) {
        return state_names_[0];
    }
    else
    if (state == VelmaLowLevelStatusSC::HW_DISABLED) {
        return state_names_[1];
    }
    else if (state == VelmaLowLevelStatusSC::HW_ENABLED) {
        return state_names_[2];
    }
    else if (state == VelmaLowLevelStatusSC::CONTROL_ENABLED) {
        return state_names_[3];
    }
    return state_names_[4];
}

VelmaLowLevelSafety::VelmaLowLevelSafety(const std::string &name) :
    TaskContext(name, PreOperational),
    arm_joints_count_(7),
    state_(VelmaLowLevelStatusSC::HW_DOWN),
    out_(*this),
    status_in_(*this),
    torso_damping_factor_(-1)  // initialize with invalid value, should be later set to >= 0
{
    this->ports()->addPort("command_INPORT", port_command_in_);
    this->ports()->addPort("status_OUTPORT", port_status_out_);

    this->ports()->addPort("status_rArm_friIntfState_INPORT", port_rArm_fri_state_in_);
    this->ports()->addPort("status_rArm_friRobotState_INPORT", port_rArm_robot_state_in_);

    this->ports()->addPort("status_lArm_friIntfState_INPORT", port_lArm_fri_state_in_);
    this->ports()->addPort("status_lArm_friRobotState_INPORT", port_lArm_robot_state_in_);

    this->ports()->addPort("cmd_rArm_cmd_OUTPORT", port_rArm_KRL_CMD_out_);
    this->ports()->addPort("cmd_lArm_cmd_OUTPORT", port_lArm_KRL_CMD_out_);

    this->ports()->addPort("port_robot_status_OUTPORT", port_robot_status_out_);

    addProperty("l_arm_damping_factors", l_arm_damping_factors_);
    addProperty("r_arm_damping_factors", r_arm_damping_factors_);
    addProperty("torso_damping_factor", torso_damping_factor_);

    addProperty("arm_q_limits_lo", arm_q_limits_lo_);
    addProperty("arm_q_limits_hi", arm_q_limits_hi_);
    addProperty("arm_dq_limits", arm_dq_limits_);
    addProperty("arm_t_limits", arm_t_limits_);

    joint_error_.resize(arm_joints_count_);
    arm_k_.resize(arm_joints_count_);
    arm_k_(0) = arm_k_(1) = arm_k_(2) = arm_k_(3) = arm_k_(4) = arm_k_(5) = arm_k_(6) = 20;

    k_.resize(arm_joints_count_);
    q_.resize(arm_joints_count_, arm_joints_count_);
    d_.resizeLike(q_);
    k0_.resizeLike(q_);
    tmpNN_.resizeLike(q_);
    es_ = Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd >(arm_joints_count_);
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

    state_ = VelmaLowLevelStatusSC::HW_DOWN;
    allHwOk_ = false;
    hwStatusValid_ = false;
    readCmdData_ = false;
    cmdValid_ = false;

    packet_counter_ = 1;

//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaLowLevelSafety::stopHook() {
}

void VelmaLowLevelSafety::calculateArmDampingTorque(const Eigen::VectorXd &joint_velocity,
    const std::vector<double> &damping_factors, Eigen::VectorXd &joint_torque_command)
{
    Logger::In in("VelmaLowLevelSafety::calculateArmDampingTorque");

    joint_torque_command.setZero();
    for (int i = 0; i < arm_joints_count_; ++i) {
        joint_torque_command(i) = -damping_factors[i] * joint_velocity(i);
    }
}

void VelmaLowLevelSafety::calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command)
{
    Logger::In in("VelmaLowLevelSafety::calculateTorsoDampingTorque");

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
    Logger::In in("VelmaLowLevelSafety::updateHook");
//    RESTRICT_ALLOC;

    //
    // read HW status
    //
    bool rArm_valid_prev = status_in_.getPorts().rArm_.valid_;
    bool lArm_valid_prev = status_in_.getPorts().lArm_.valid_;

    status_in_.readPorts(status_);

    bool allHwOk_prev = allHwOk_;
    bool readCmdData_prev = readCmdData_;
    bool cmdValid_prev = cmdValid_;

    // as FRI components are not synchronized, their communication shatus should
    // be checked in two last cycles
    bool rArm_valid = rArm_valid_prev || status_in_.getPorts().rArm_.valid_;
    bool lArm_valid = lArm_valid_prev || status_in_.getPorts().lArm_.valid_;

    // check FRI and LWR state
    // as FRI components may not be synchronized
    if (port_rArm_fri_state_in_.read(rArm_fri_state_) == RTT::NewData && port_rArm_robot_state_in_.read(rArm_robot_state_) == RTT::NewData) {
        rArm_valid &= isLwrOk(rArm_fri_state_, rArm_robot_state_);
    }
    if (port_lArm_fri_state_in_.read(lArm_fri_state_) == RTT::NewData && port_lArm_robot_state_in_.read(lArm_robot_state_) == RTT::NewData) {
        lArm_valid &= isLwrOk(lArm_fri_state_, lArm_robot_state_);
    }
    rArm_valid &= isStatusValid(status_.rArm);
    lArm_valid &= isStatusValid(status_.lArm);

    bool rHand_valid = status_in_.getPorts().rHand_.valid_ && isStatusValid(status_.rHand);
    bool lHand_valid = status_in_.getPorts().lHand_.valid_ && isStatusValid(status_.lHand);

    bool rFt_valid = status_in_.getPorts().rFt_.valid_ && isStatusValid(status_.rFt);
    bool lFt_valid = status_in_.getPorts().lFt_.valid_ && isStatusValid(status_.lFt);

    bool tMotor_valid = status_in_.getPorts().tMotor_.valid_ && isStatusValid(status_.tMotor);
    bool hpMotor_valid = status_in_.getPorts().hpMotor_.valid_ && isStatusValid(status_.hpMotor);
    bool htMotor_valid = status_in_.getPorts().htMotor_.valid_ && isStatusValid(status_.htMotor);

    allHwOk_ =  rArm_valid      && lArm_valid &&
                rHand_valid     && lHand_valid &&
                rFt_valid       && lFt_valid &&
                tMotor_valid    && hpMotor_valid &&
                htMotor_valid;

    // send diagnostic information about robot state
    std_msgs::UInt32 robot_state;
    robot_state.data =  rArm_valid          | (lArm_valid<<1) |
                        (rHand_valid<<2)    | (lHand_valid<<3) |
                        (rFt_valid<<4)      | (lFt_valid<<5) |
                        (tMotor_valid<<6)   | (hpMotor_valid<<7) |
                        (htMotor_valid<<8);

    port_robot_status_out_.write(robot_state);

    //
    // read commands
    //
    readCmdData_ = (port_command_in_.read(cmd_in_) == NewData);
    cmdValid_ = false;
    if (readCmdData_ && isCommandValid(cmd_in_)) {
        cmdValid_ = true;
    }

    if (readCmdData_ && cmd_in_.test != packet_counter_ && VelmaLowLevelStatusSC::HW_DOWN != state_) {
        Logger::log() << Logger::Warning << "received wrong cmd frame: " << cmd_in_.test << " should be " << packet_counter_ << Logger::endl;
    }


    const int32_t prev_state = state_;

    if (allHwOk_prev != allHwOk_ || allHwOk_prev != allHwOk_ || readCmdData_prev != readCmdData_ ||
        cmdValid_prev != cmdValid_) {

        Logger::log() << Logger::Info << "state: " << getStateName(state_) <<
            "  allHwOk: " << (allHwOk_?"T":"F") <<
            "  readCmdData: " << (readCmdData_?"T":"F") <<
            "  cmdValid: " << (cmdValid_?"T":"F") << Logger::endl;
        Logger::log() << Logger::Info << "cmd: " << cmdToStr(cmd_in_) << Logger::endl;
    }

    if (cmd_in_.sc.valid) {
        if (cmd_in_.sc.cmd == 1) {
            Logger::log() << Logger::Info << "received cmd: enable_hw" << Logger::endl;
        }
        else if (cmd_in_.sc.cmd == 2) {
            Logger::log() << Logger::Info << "received cmd: enable_control" << Logger::endl;
        }
        else {
            Logger::log() << Logger::Info << "received wrong cmd: " << cmd_in_.sc.cmd << Logger::endl;
        }
    }

    //
    // manage FSM state transitions
    //
    if (VelmaLowLevelStatusSC::HW_DOWN == state_) {
        if (allHwOk_) {
            state_ = VelmaLowLevelStatusSC::HW_DISABLED;
        }
    }
    else if (VelmaLowLevelStatusSC::HW_DISABLED == state_) {
        // state changes
        if (!allHwOk_) {
            // one or more HW components are down
            state_ = VelmaLowLevelStatusSC::HW_DOWN;
        }
        else if (cmd_in_.sc.valid && cmd_in_.sc.cmd == 1) {
            state_ = VelmaLowLevelStatusSC::HW_ENABLED;
            Logger::log() << Logger::Info << "accepted cmd: enable_hw" << Logger::endl;
        }
    }
    else if (VelmaLowLevelStatusSC::HW_ENABLED == state_) {
        if (!allHwOk_) {
            state_ = VelmaLowLevelStatusSC::HW_DOWN;
        }
        else if (cmdValid_ && cmd_in_.sc.valid && cmd_in_.sc.cmd == 2) {
            // change state to HW_ENABLED
            state_ = VelmaLowLevelStatusSC::CONTROL_ENABLED;
            Logger::log() << Logger::Info << "accepted cmd: enable_control" << Logger::endl;
        }
    }
    else if (VelmaLowLevelStatusSC::CONTROL_ENABLED == state_) {
        if (!allHwOk_) {
            state_ = VelmaLowLevelStatusSC::HW_DOWN;
        }
        else if (!cmdValid_) {
            state_ = VelmaLowLevelStatusSC::HW_ENABLED;
        }
    }

    if (prev_state != state_) {
        Logger::log() << Logger::Info << "state change: " << getStateName(prev_state) << " -> " << getStateName(state_) << Logger::endl;
    }

    //
    // execute states behaviours
    //
    if (VelmaLowLevelStatusSC::HW_DOWN == state_) {
        //
        // write HW commands to available devices
        //

        // generate safe outputs for all operational devices
        if (rArm_valid) {
            arm_dq_.convertFromROS(status_.rArm.dq);
            calculateArmDampingTorque(arm_dq_.data_, r_arm_damping_factors_, arm_t_cmd_.data_);
            arm_t_cmd_.convertToROS(cmd_out_.rArm.t);
            out_.getPorts().rArm_.convertFromROS(cmd_out_.rArm);
            out_.getPorts().rArm_.writePorts();
        }

        if (lArm_valid) {
            arm_dq_.convertFromROS(status_.lArm.dq);
            calculateArmDampingTorque(arm_dq_.data_, l_arm_damping_factors_, arm_t_cmd_.data_);
            arm_t_cmd_.convertToROS(cmd_out_.lArm.t);
            out_.getPorts().lArm_.convertFromROS(cmd_out_.lArm);
            out_.getPorts().lArm_.writePorts();
        }

        if (tMotor_valid) {
            calculateTorsoDampingTorque(status_.tMotor.dq, cmd_out_.tMotor.i);
            cmd_out_.tMotor.q = 0;
            cmd_out_.tMotor.dq = 0;
            out_.getPorts().tMotor_.convertFromROS(cmd_out_.tMotor);
            out_.getPorts().tMotor_.writePorts();
        }

        if (hpMotor_valid) {
            cmd_out_.hpMotor.i = 0;
            cmd_out_.hpMotor.q = status_.hpMotor.q;
            cmd_out_.hpMotor.dq = 0;
            out_.getPorts().hpMotor_.convertFromROS(cmd_out_.hpMotor);
            out_.getPorts().hpMotor_.writePorts();
        }

        if (htMotor_valid) {
            cmd_out_.htMotor.i = 0;
            cmd_out_.htMotor.q = status_.htMotor.q;
            cmd_out_.htMotor.dq = 0;
            out_.getPorts().htMotor_.convertFromROS(cmd_out_.htMotor);
            out_.getPorts().htMotor_.writePorts();
        }
    }
    else if (VelmaLowLevelStatusSC::HW_DISABLED == state_ || VelmaLowLevelStatusSC::HW_ENABLED == state_) {
        arm_dq_.convertFromROS(status_.rArm.dq);
        calculateArmDampingTorque(arm_dq_.data_, r_arm_damping_factors_, arm_t_cmd_.data_);
        arm_t_cmd_.convertToROS(cmd_out_.rArm.t);

        arm_dq_.convertFromROS(status_.lArm.dq);
        calculateArmDampingTorque(arm_dq_.data_, l_arm_damping_factors_, arm_t_cmd_.data_);
        arm_t_cmd_.convertToROS(cmd_out_.lArm.t);

        calculateTorsoDampingTorque(status_.tMotor.dq, cmd_out_.tMotor.i);

        cmd_out_.hpMotor.i = 0;
        cmd_out_.hpMotor.q = status_.hpMotor.q;
        cmd_out_.hpMotor.dq = 0;

        cmd_out_.htMotor.i = 0;
        cmd_out_.htMotor.q = status_.htMotor.q;
        cmd_out_.htMotor.dq = 0;

        cmd_out_.rTact.cmd = 0;
        cmd_out_.rTact.valid = false;
        cmd_out_.rHand.valid = false;
        cmd_out_.lHand.valid = false;

        //
        // write HW commands
        //
        out_.writePorts(cmd_out_);

        //
        // write status
        //
        status_.sc.state_id = static_cast<int32_t >(state_);
        status_.sc.error_code = 0;
        ++packet_counter_;
        status_.test = packet_counter_;
        port_status_out_.write(status_);

        if (VelmaLowLevelStatusSC::HW_ENABLED == state_ && cmd_in_.sc.valid && cmd_in_.sc.cmd == 1) {
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
    else if (VelmaLowLevelStatusSC::CONTROL_ENABLED == state_) {
        //
        // write HW commands
        //
        out_.writePorts(cmd_in_);

        //
        // write status
        //
        status_.sc.state_id = static_cast<int32_t >(state_);
        status_.sc.error_code = 0;
        ++packet_counter_;
        status_.test = packet_counter_;
        port_status_out_.write(status_);
    }
}

ORO_LIST_COMPONENT_TYPE(VelmaLowLevelSafety)
ORO_CREATE_COMPONENT_LIBRARY();

