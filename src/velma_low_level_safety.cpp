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

    const int number_of_joints = 7;
    joint_error_.resize(number_of_joints);
    arm_k_.resize(number_of_joints);
    arm_k_(0) = arm_k_(1) = arm_k_(2) = arm_k_(3) = arm_k_(4) = arm_k_(5) = arm_k_(6) = 20;

    k_.resize(number_of_joints);
    q_.resize(number_of_joints, number_of_joints);
    d_.resizeLike(q_);
    k0_.resizeLike(q_);
    tmpNN_.resizeLike(q_);
    es_ = Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd >(number_of_joints);
}

bool VelmaLowLevelSafety::configureHook() {
    Logger::In in("VelmaLowLevelSafety::configureHook");

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

bool VelmaLowLevelSafety::calculateJntImpTorque(const Eigen::VectorXd &joint_position_command,
    const Eigen::VectorXd &joint_position, const Eigen::VectorXd &joint_velocity,
    const Eigen::VectorXd &k, const Eigen::Matrix77d &m,
    Eigen::VectorXd &joint_torque_command)
{
    Logger::In in("VelmaLowLevelSafety::calculateJntImpTorque");

    joint_error_.noalias() = joint_position_command - joint_position;
    joint_torque_command.noalias() = k.cwiseProduct(joint_error_);

    if (!joint_torque_command.allFinite()) {
        Logger::log() << Logger::Error << "Non finite output from stiffness" << std::endl;
        return false;
    }

    tmpNN_ = k.asDiagonal();
    es_.compute(tmpNN_, m);
    q_ = es_.eigenvectors().inverse();
    k0_ = es_.eigenvalues();

    tmpNN_ = k0_.cwiseAbs().cwiseSqrt().asDiagonal();

    d_.noalias() = 2.0 * q_.transpose() * 0.7 * tmpNN_ * q_;
    joint_torque_command.noalias() -= d_ * joint_velocity;
    if (!joint_torque_command.allFinite()) {
        Logger::log() << Logger::Error << "Non finite output from damping: s_q: " << joint_position_command.transpose() <<
            " q: " << joint_position.transpose() << " dq: " << joint_velocity.transpose() <<
            " k: " << k.transpose() << " m: " << m << std::endl;
        return false;
    }
    return true;
}

void VelmaLowLevelSafety::updateHook() {
    Logger::In in("VelmaLowLevelSafety::updateHook");
//    RESTRICT_ALLOC;

    bool status_valid = false;
    if (port_status_in_.read(status_in_) == NewData) {
        // verify robot status here
        status_valid = true;
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
        // emergency procedure
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

        for (int i = 0; i < 4; ++i) {
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

        if (!emergency_) {
            emergency_ = true;
            rArm_safe_q_.convertFromROS(status_in_.rArm.q);
            lArm_safe_q_.convertFromROS(status_in_.lArm.q);
            Logger::log() << Logger::Info << "sending emergency data..." << Logger::endl;
        }
        else {
            Logger::log() << Logger::Debug << "sending emergency data" << Logger::endl;
        }

        if (status_valid) {
            // right arm
            arm_q_.convertFromROS(status_in_.rArm.q);
            arm_dq_.convertFromROS(status_in_.rArm.dq);

            arm_mass77_.convertFromROS(status_in_.rArm.mmx);

            if ( calculateJntImpTorque(rArm_safe_q_.data_, arm_q_.data_, arm_dq_.data_,
                arm_k_, arm_mass77_.data_, arm_t_cmd_.data_) )
            {
                arm_t_cmd_.convertToROS(cmd_out_.rArm.t);
            }

            // left arm
            arm_q_.convertFromROS(status_in_.lArm.q);
            arm_dq_.convertFromROS(status_in_.lArm.dq);

            arm_mass77_.convertFromROS(status_in_.lArm.mmx);

            if ( calculateJntImpTorque(lArm_safe_q_.data_, arm_q_.data_, arm_dq_.data_,
                arm_k_, arm_mass77_.data_, arm_t_cmd_.data_) )
            {
                arm_t_cmd_.convertToROS(cmd_out_.lArm.t);
            }
        }

        out_.writePorts(cmd_out_);
    }
}

ORO_LIST_COMPONENT_TYPE(VelmaLowLevelSafety)
ORO_CREATE_COMPONENT_LIBRARY();

