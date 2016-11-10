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

bool VelmaLowLevelSafety::isNaN(double d) const {
    return d != d;
}

bool VelmaLowLevelSafety::isInLim(double d, double lo_lim, double hi_lim) const {
    return d == d && d > lo_lim && d < hi_lim;
}

bool VelmaLowLevelSafety::isCommandValidTorso(const VelmaLowLevelCommandMotor &cmd, int& idx) const {
// TODO
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isCommandValidHeadPan(const VelmaLowLevelCommandMotor &cmd, int& idx) const {
// TODO
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isCommandValidHeadTilt(const VelmaLowLevelCommandMotor &cmd, int& idx) const {
// TODO
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isCommandValid(const VelmaLowLevelCommandArm &cmd, int& idx) const {
    for (int i = 0; i < arm_joints_count_; ++i) {
        if (!isInLim(cmd.t[i], -arm_t_limits_[i], arm_t_limits_[i]))
        {
            idx = i+1;
            return false;
        }
    }
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isCommandValid(const VelmaLowLevelCommandHand &cmd, int& idx) const {
// TODO
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isCommandValid(const VelmaLowLevelCommand &cmd, int& module, int& submodule) const {
    if ( !isCommandValid(cmd.lArm, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_L_ARM;
        return false;
    }

    if ( !isCommandValid(cmd.rArm, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_R_ARM;
        return false;
    }

    if ( !isCommandValid(cmd.lHand, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_L_HAND;
        return false;
    }

    if ( !isCommandValid(cmd.rHand, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_R_HAND;
        return false;
    }

    if ( !isCommandValidTorso(cmd.tMotor, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_T_MOTOR;
        return false;
    }

    if ( !isCommandValidHeadPan(cmd.hpMotor, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_HP_MOTOR;
        return false;
    }

    if ( !isCommandValidHeadTilt(cmd.htMotor, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_HT_MOTOR;
        return false;
    }

    module = -1;

    return true;
}

bool VelmaLowLevelSafety::isStatusValid(const VelmaLowLevelStatusArm &st, int& idx) const {
    for (int i = 0; i < arm_joints_count_; ++i) {
        if (!isInLim(st.q[i], arm_q_limits_lo_[i], arm_q_limits_hi_[i]) ||
            isNaN(st.dq[i]) ||
            isNaN(st.t[i]) ||
            isNaN(st.gt[i]))
        {
            idx = i+1;
            return false;
        }
    }

// TODO:
//    isInLim(st.w.force.x, 
//    st.mmx

    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isStatusValid(const VelmaLowLevelStatusHand &st, int& idx) const {
    // TODO
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isStatusValid(const VelmaLowLevelStatusMotor &st, int& idx) const {
    // TODO
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isStatusValid(const VelmaLowLevelStatusFT &st, int& idx) const {
    // TODO
    idx = 0;
    return true;
}

bool VelmaLowLevelSafety::isStatusValid(const VelmaLowLevelStatus &st, int& module, int& submodule) const {
// TODO:
// barrett_hand_controller_msgs/BHPressureState rHand_p
// geometry_msgs/WrenchStamped[3] lHand_f

    if ( !isStatusValid(st.lArm, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_L_ARM;
        return false;
    }

    if ( !isStatusValid(st.rArm, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_R_ARM;
        return false;
    }

    if ( !isStatusValid(st.lHand, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_L_HAND;
        return false;
    }

    if ( !isStatusValid(st.rHand, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_R_HAND;
        return false;
    }

    if ( !isStatusValid(st.lFt, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_L_FT;
        return false;
    }

    if ( !isStatusValid(st.rFt, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_R_FT;
        return false;
    }

    if ( !isStatusValid(st.tMotor, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_T_MOTOR;
        return false;
    }

    if ( !isStatusValid(st.hpMotor, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_HP_MOTOR;
        return false;
    }

    if ( !isStatusValid(st.htMotor, submodule) ) {
        module = VelmaLowLevelStatusSC::MODULE_HT_MOTOR;
        return false;
    }

    module = 0;

    return true;
}

bool VelmaLowLevelSafety::isLwrOk(const tFriIntfState &fri_state, const tFriRobotState &robot_state) const {
    if (robot_state.power != 0x7F || robot_state.error != 0 ||    // error
        robot_state.warning != 0 ||                                // error?
        robot_state.control != FRI_CTRL_JNT_IMP ||                 // error
        fri_state.quality <= FRI_QUALITY_UNACCEPTABLE)             // error
    {
        // error
        return false;
    }
    return true;
}

