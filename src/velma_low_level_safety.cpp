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

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>

#include "velma_low_level_safety.h"

using namespace RTT;

VelmaLowLevelSafety::VelmaLowLevelSafety(const std::string &name) :
    TaskContext(name, PreOperational),
    out_(*this)
{
    this->ports()->addPort("command_INPORT", port_command_in_);
    this->ports()->addPort("status_INPORT", port_status_in_);
}

bool VelmaLowLevelSafety::configureHook() {
    return true;
}

bool VelmaLowLevelSafety::startHook() {
//    RESTRICT_ALLOC;

    emergency_ = false;
//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaLowLevelSafety::stopHook() {
}

void VelmaLowLevelSafety::updateHook() {
    Logger::In in("VelmaLowLevelSafety::updateHook");
//    RESTRICT_ALLOC;

    if (port_status_in_.read(status_in_) == NewData) {
        // verify robot status here
    }

    uint32_t comm_status_in = 0;
    if (port_command_in_.read(cmd_in_) == NewData) {
        out_.writePorts(cmd_in_);
        if (emergency_) {
            emergency_ = false;
            Logger::log() << Logger::Info << "sending valid data..." << Logger::endl;
        }
        else {
            Logger::log() << Logger::Debug << "sending valid data..." << Logger::endl;
        }
    }
    else {
        cmd_out_.rHand_tactileCmd = 0;
        cmd_out_.tMotor_i = 0;
        cmd_out_.hpMotor_i = 0;
        cmd_out_.htMotor_i = 0;
        cmd_out_.hpMotor_q = 0;
        cmd_out_.htMotor_q = 0;
        cmd_out_.hpMotor_dq = 0;
        cmd_out_.htMotor_dq = 0;
        for (int i = 0; i < 7; ++i) {
            cmd_out_.rArm.t[i] = 0.0;
            cmd_out_.lArm.t[i] = 0.0;
        }
        cmd_out_.rArm.cmd.data = 0;
        cmd_out_.lArm.cmd.data = 0;

        for (int i = 0; i < 7; ++i) {
            cmd_out_.rHand.q[i] = 0;
            cmd_out_.rHand.dq[i] = 0;
            cmd_out_.rHand.max_p[i] = 0;
            cmd_out_.rHand.max_i[i] = 0;
            cmd_out_.lHand.q[i] = 0;
            cmd_out_.lHand.dq[i] = 0;
            cmd_out_.lHand.max_p[i] = 0;
            cmd_out_.lHand.max_i[i] = 0;
        }
        cmd_out_.rHand.hold = false;
        cmd_out_.lHand.hold = false;

        out_.writePorts(cmd_out_);
        if (!emergency_) {
            emergency_ = true;
            Logger::log() << Logger::Info << "sending emergency data..." << Logger::endl;
        }
        else {
            Logger::log() << Logger::Debug << "sending emergency data" << Logger::endl;
        }
    }
}

ORO_LIST_COMPONENT_TYPE(VelmaLowLevelSafety)
ORO_CREATE_COMPONENT_LIBRARY();

