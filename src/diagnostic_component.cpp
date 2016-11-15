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

#include "diagnostic_component.h"
#include <math.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <sys/time.h>

#include "common.h"

using namespace RTT;

static std::string getTaskStatusChar(RTT::TaskContext* t)
{
    if (t->inFatalError())
        return "F";
    if (t->inRunTimeError())
        return "E";
    if (t->inException())
        return "X";
    if (t->isRunning() )
        return "R"; // Running
    if (t->isConfigured() )
        return "S"; // Stopped
    return "U";     // Unconfigured/Preoperational
}

static std::string faultTypeToString(int fault_type) {
    switch (fault_type) {
    case VelmaLowLevelStatusSC::FAULT_NAN_COMMAND:
        return "FAULT_NAN_COMMAND";
    case VelmaLowLevelStatusSC::FAULT_RANGE_COMMAND:
        return "FAULT_RANGE_COMMAND";
    case VelmaLowLevelStatusSC::FAULT_NAN_STATUS:
        return "FAULT_NAN_STATUS";
    case VelmaLowLevelStatusSC::FAULT_RANGE_STATUS:
        return "FAULT_RANGE_STATUS";
    case VelmaLowLevelStatusSC::FAULT_COMM_HW:
        return "FAULT_COMM_HW";
    case VelmaLowLevelStatusSC::FAULT_COMM_UP:
        return "FAULT_COMM_UP";
    case VelmaLowLevelStatusSC::FAULT_COMM_PACKET_LOST:
        return "FAULT_COMM_PACKET_LOST";
    case VelmaLowLevelStatusSC::FAULT_HW_STATE:
        return "FAULT_HW_STATE";
    }
    return "FAULT_UNKNOWN";
}

static std::string moduleIdToString(int module_id) {
    switch (module_id) {
    case VelmaLowLevelStatusSC::MODULE_R_ARM:
        return "MODULE_R_ARM";
    case VelmaLowLevelStatusSC::MODULE_L_ARM:
        return "MODULE_L_ARM";
    case VelmaLowLevelStatusSC::MODULE_R_HAND:
        return "MODULE_R_HAND";
    case VelmaLowLevelStatusSC::MODULE_L_HAND:
        return "MODULE_L_HAND";
    case VelmaLowLevelStatusSC::MODULE_R_FT:
        return "MODULE_R_FT";
    case VelmaLowLevelStatusSC::MODULE_L_FT:
        return "MODULE_L_FT";
    case VelmaLowLevelStatusSC::MODULE_T_MOTOR:
        return "MODULE_T_MOTOR";
    case VelmaLowLevelStatusSC::MODULE_HP_MOTOR:
        return "MODULE_HP_MOTOR";
    case VelmaLowLevelStatusSC::MODULE_HT_MOTOR:
        return "MODULE_HT_MOTOR";
    case VelmaLowLevelStatusSC::MODULE_R_TACTILE:
        return "MODULE_R_TACTILE";
    case VelmaLowLevelStatusSC::MODULE_L_OPTOFORCE:
        return "MODULE_L_OPTOFORCE";
    }
    return "MODULE_UNKNOWN";
}

DiagnosticComponent::DiagnosticComponent(const std::string &name) :
    TaskContext(name, PreOperational),
    port_diag_out_("diag_OUTPORT", true),
    port_status_sc_in_("status_sc_INPORT")
{
    this->ports()->addPort(port_diag_out_);
    this->ports()->addPort(port_status_sc_in_);
}

bool DiagnosticComponent::configureHook() {
    Logger::In in("DiagnosticComponent::configureHook");

    TaskContext::PeerList l = this->getPeerList();

    diag_out_.status.resize(2);

    diag_out_.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_out_.status[0].name = "components";
    diag_out_.status[0].hardware_id = "0";
    diag_out_.status[0].message = "";
    diag_out_.status[0].values.resize(l.size());

    for (int i = 0; i < l.size(); ++i) {
        TaskContext *tc = this->getPeer( l[i] );
        if (tc == NULL) {
            Logger::log() << Logger::Error << "could not find peer "
                          << l[i] << Logger::endl;
            return false;
        }
        peers_.push_back(tc);
        diag_out_.status[0].values[i].key = l[i];          // component name
        diag_out_.status[0].values[i].value = "UNKNOWN";   // component state
    }

    diag_out_.status[1].level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_out_.status[1].name = "safety_controller";
    diag_out_.status[1].hardware_id = "1";
    diag_out_.status[1].message = "";
    diag_out_.status[1].values.resize(2);

    diag_out_.status[1].values[0].key = "state";
    diag_out_.status[1].values[0].value = "UNKNOWN";

    diag_out_.status[1].values[1].key = "fault";
    diag_out_.status[1].values[1].value = "UNKNOWN";

    return true;
}

bool DiagnosticComponent::startHook() {
    diag_out_.header.seq = 0;
    return true;
}

void DiagnosticComponent::stopHook() {
}

void DiagnosticComponent::updateHook() {
    port_status_sc_in_.read(status_sc_in_);

    bool changed = true;//false;

    // states of components
    for (int i = 0; i < peers_.size(); ++i) {
        std::string value = getTaskStatusChar( peers_[i] );
        if (value != diag_out_.status[0].values[i].value) {
            diag_out_.status[0].values[i].value = value;  // component state
            changed = true;
        }
    }

    // safety controller state
    const std::string& state_name = getStateName(status_sc_in_.state_id);
    if (diag_out_.status[1].values[0].value != state_name) {
        diag_out_.status[1].values[0].value = state_name;
        changed = true;
    }

    std::string prev_fault = diag_out_.status[1].values[1].value;
    if (status_sc_in_.error) {
        ostringstream ss;
        ss << faultTypeToString(status_sc_in_.fault_type) << " "
           << moduleIdToString(status_sc_in_.faulty_module_id)
           << " " << status_sc_in_.faulty_submodule_id;
        diag_out_.status[1].values[1].value = ss.str();
    }
    else {
        diag_out_.status[1].values[1].value = "NONE";
    }

    if (diag_out_.status[1].values[1].value != prev_fault) {
        changed = true;
    }

    if (changed) {
        diag_out_.header.seq++;
        diag_out_.header.stamp = ros::Time::now();
        port_diag_out_.write(diag_out_);
    }
}

ORO_LIST_COMPONENT_TYPE(DiagnosticComponent)

