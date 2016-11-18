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
#include <algorithm>

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>

#include "velma_low_level_master.h"
#include <math.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <sys/time.h>

using namespace RTT;

VelmaLowLevelMaster::VelmaLowLevelMaster(const std::string &name) :
    TaskContext(name, PreOperational),
    cmd_ports_out_(*this),
    status_ports_in_(*this)
{
    this->ports()->addPort("command_INPORT", port_command_in_);
    this->ports()->addPort("status_sc_OUTPORT", port_status_sc_out_);
    this->ports()->addPort("status_test_OUTPORT", port_status_test_out_);
}

bool VelmaLowLevelMaster::configureHook() {
    Logger::In in("VelmaLowLevelMaster::configureHook");

    // TODO: read those parameters from rosparam
    state_names_.push_back("idle");
    state_names_.push_back("safe");
    initial_state_name_ = "safe";


    // retrieve states list
    for (int i = 0; i < state_names_.size(); ++i) {
        auto b_ptr = StateFactory<VelmaLowLevelStatus, VelmaLowLevelCommand >::Instance()->Create( state_names_[i] );
        if (b_ptr) {
            states_.push_back(b_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown state: " << state_names_[i] << Logger::endl;
            return false;
        }
    }

    // retrieve behavior names
    for (int i = 0; i < states_.size(); ++i) {
        const std::string& behavior_name = states_[i]->getBehaviorName();
        bool add = true;
        for (int j = 0; j < behavior_names_.size(); ++j) {
            if (behavior_names_[j] == behavior_name) {
                add = false;
                break;
            }
        }
        if (add) {
            behavior_names_.push_back(behavior_name);
        }
    }

    // retrieve behaviors list
    for (int i = 0; i < behavior_names_.size(); ++i) {
        auto b_ptr = BehaviorFactory<VelmaLowLevelStatus, VelmaLowLevelCommand >::Instance()->Create( behavior_names_[i] );
        if (b_ptr) {
            behaviors_.push_back(b_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown behavior: " << behavior_names_[i] << Logger::endl;
            return false;
        }
    }

    // select initial state
    for (int i = 0; i < states_.size(); ++i) {
        if (states_[i]->getStateName() == initial_state_name_) {
            current_state_ = states_[i];
        }
    }

    if (!current_state_) {
        Logger::log() << Logger::Error << "unknown initial state: " << initial_state_name_ << Logger::endl;
        return false;
    }

    // get names of all components that are needed for all behaviors
    std::set<std::string > switchable_components;

    for (int i = 0; i < behaviors_.size(); ++i) {
        const std::vector<std::string >& comp_vec = behaviors_[i]->getRunningComponents();
        for (int j = 0; j < comp_vec.size(); ++j) {
            switchable_components.insert( comp_vec[j] );
        }
    }






    TaskContext::PeerList l = this->getPeerList();
    if (l.size() != 1) {
        Logger::log() << Logger::Error << "wrong number of peers: " << l.size() << ", should be 1" << Logger::endl;
        return false;
    }

    TaskContext::PeerList::const_iterator it = l.begin();
    scheme_ = this->getPeer( (*it) );

    RTT::OperationInterfacePart *hasBlockOp = scheme_->getOperation("hasBlock");
    if (hasBlockOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation hasBlock" << Logger::endl;
        return false;
    }

    hasBlock_ =  RTT::OperationCaller<bool(const std::string &)>(
        hasBlockOp, scheme_->engine());


    for (std::set<std::string >::const_iterator it = switchable_components.begin(); it != switchable_components.end(); ++it) {
        if (!hasBlock_( *it )) {
            Logger::log() << Logger::Error << "could not find a component: " << (*it) << " in the scheme blocks list" << Logger::endl;
            return false;
        }
    }

    RTT::OperationInterfacePart *addGraphConfigurationOp = scheme_->getOperation("addGraphConfiguration");
    if (addGraphConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation addGraphConfiguration" << Logger::endl;
        return false;
    }

    addGraphConfiguration_ = RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)>(
        addGraphConfigurationOp, scheme_->engine());

    RTT::OperationInterfacePart *switchToConfigurationOp = scheme_->getOperation("switchToConfiguration");
    if (switchToConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation switchToConfiguration" << Logger::endl;
        return false;
    }

    switchToConfiguration_ = RTT::OperationCaller<bool(int)>(
        switchToConfigurationOp, scheme_->engine());

    // add graph configuration for each behavior
    for (int i = 0; i < behaviors_.size(); ++i) {
        std::vector<std::string > vec_stopped;
        const std::vector<std::string >& vec_running = behaviors_[i]->getRunningComponents();
        for (std::set<std::string >::const_iterator ic = switchable_components.begin(); ic != switchable_components.end(); ++ic) {
            bool is_running = false;
            for (int ir = 0; ir < vec_running.size(); ++ir) {
                if ( (*ic) == vec_running[ir] ) {
                    is_running = true;
                    break;
                }
            }
            if (!is_running) {
                vec_stopped.push_back( *ic );
            }
        }
        addGraphConfiguration_(i, vec_stopped, vec_running);
    }

    // retrieve the vector of peers of conman scheme
    TaskContext::PeerList scheme_peers_names = scheme_->getPeerList();
    for (int pi = 0; pi < scheme_peers_names.size(); ++pi) {
        scheme_peers_.push_back( scheme_->getPeer(scheme_peers_names[pi]) );
    }

    return true;
}

bool VelmaLowLevelMaster::startHook() {
    return true;
}

void VelmaLowLevelMaster::stopHook() {
}

void VelmaLowLevelMaster::updateHook() {
    // reset error status
    status_sc_out_.error = false;
    status_sc_out_.fault_type = 0;
    status_sc_out_.faulty_module_id = 0;
    status_sc_out_.faulty_submodule_id = 0;

    int id_faulty_module;
    int id_faulty_submodule;

    //
    // read HW status (from previous iteration)
    //
    status_ports_in_.readPorts();
    status_ports_in_.convertToROS(status_in_);

    //
    // read commands
    //
    bool cmd_in_valid = (port_command_in_.read(cmd_in_) == NewData);

    // get current behavior
    std::shared_ptr<BehaviorBase<VelmaLowLevelStatus, VelmaLowLevelCommand> > current_behavior;
    for (int i = 0; i < behaviors_.size(); ++i) {
        if (current_state_->getBehaviorName() == behaviors_[i]->getName()) {
            current_behavior = behaviors_[i];
            break;
        }
    }

    bool state_switch = false;

    //
    // check error condition
    //
    bool pred_err = false;
    pred_err = current_behavior->checkErrorCondition(status_in_, cmd_in_, scheme_peers_);

    if (pred_err) {
        int next_state_index = -1;
        for (int i = 0; i < states_.size(); ++i) {
            if ( states_[i]->checkInitialCondition(status_in_, cmd_in_, scheme_peers_, current_state_->getStateName(), true) ) {
                if (next_state_index == -1) {
                    next_state_index = i;
                }
                else {
                    Logger::In in("VelmaLowLevelMaster::updateHook");
                    Logger::log() << Logger::Error << "two or more states have the same initial condition (err): current_state="
                        << current_state_->getStateName() << Logger::endl;
                    error();
                }
            }
        }
        if (next_state_index == -1) {
            Logger::In in("VelmaLowLevelMaster::updateHook");
            Logger::log() << Logger::Error << "cannot switch to new state (initial condition, err): current_state="
                << current_state_->getStateName() << Logger::endl;
        }
        else {
            current_state_ = states_[next_state_index];
            state_switch = true;
        }
    }
    else {
        //
        // check stop condition
        //
        bool pred_stop = false;
        pred_stop = current_behavior->checkStopCondition(status_in_, cmd_in_, scheme_peers_);

        if (pred_stop) {
            int next_state_index = -1;
            for (int i = 0; i < states_.size(); ++i) {
                if ( states_[i]->checkInitialCondition(status_in_, cmd_in_, scheme_peers_, current_state_->getStateName(), false) ) {
                    if (next_state_index == -1) {
                        next_state_index = i;
                    }
                    else {
                        Logger::In in("VelmaLowLevelMaster::updateHook");
                        Logger::log() << Logger::Error << "two or more states have the same initial condition (stop): current_state="
                            << current_state_->getStateName() << Logger::endl;
                        error();
                    }
                }
            }
            if (next_state_index == -1) {
                Logger::In in("VelmaLowLevelMaster::updateHook");
                Logger::log() << Logger::Error << "cannot switch to new state (initial condition, stop): current_state="
                    << current_state_->getStateName() << Logger::endl;
            }
            else {
                current_state_ = states_[next_state_index];
                state_switch = true;
            }
        }
    }

    //
    // if the state has changed, reorganize the graph
    //
    if (state_switch) {
        const std::string& behavior_name = current_state_->getBehaviorName();

        for (int i = 0; i < behaviors_.size(); ++i) {
            if (behaviors_[i]->getName() == behavior_name) {
                switchToConfiguration_(i);
                break;
            }
        }
    }


/*


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
//        Logger::In in("VelmaLowLevelMaster::updateHook");
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
*/
}

ORO_LIST_COMPONENT_TYPE(VelmaLowLevelMaster)

