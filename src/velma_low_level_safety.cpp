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

    no_hw_error_counter_ = 0;

    // TODO: set this to false
    enable_command_mode_switch_ = true;

    lArm_fri_state_ = FRI_STATE_INVALID;
    rArm_fri_state_ = FRI_STATE_INVALID;

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

void VelmaLowLevelSafety::calculateArmDampingTorque(const Eigen::VectorXd &joint_velocity,
    Eigen::VectorXd &joint_torque_command)
{
    Logger::In in("VelmaLowLevelSafety::calculateArmDampingTorque");

    joint_torque_command.setZero();

    // TODO: read those parameters from rosparam
    joint_torque_command(0) = -10 * joint_velocity(0);
    joint_torque_command(1) = -10 * joint_velocity(1);
    joint_torque_command(2) = -5 * joint_velocity(2);
    joint_torque_command(3) = -5 * joint_velocity(3);
    joint_torque_command(4) = -2 * joint_velocity(4);
    joint_torque_command(5) = -2 * joint_velocity(5);
    joint_torque_command(6) = -1 * joint_velocity(6);
}

bool VelmaLowLevelSafety::is_command_valid(const VelmaLowLevelCommand &cmd) {
//    cmd.test = seed;

    // check command code
//    cmd.rHand_tactileCmd = static_cast<int32_t >(rand());

    // test for NaN
    if (cmd.tMotor_i != cmd.tMotor_i) {
        return false;
    }

    if (cmd.hpMotor_i != cmd.hpMotor_i) {
        return false;
    }

    if (cmd.htMotor_i != cmd.htMotor_i) {
        return false;
    }

    if (cmd.hpMotor_q != cmd.hpMotor_q) {
        return false;
    }

    if (cmd.htMotor_q != cmd.htMotor_q) {
        return false;
    }

    if (cmd.hpMotor_dq != cmd.hpMotor_dq) {
        return false;
    }

    if (cmd.htMotor_dq != cmd.htMotor_dq) {
        return false;
    }

    if (cmd.lArm.t.size() != 7) {
        return false;
    }

    if (cmd.rArm.t.size() != 7) {
        return false;
    }

    for (int i = 0; i < 7; ++i) {
        if (cmd.lArm.t[i] != cmd.lArm.t[i]) {
            return false;
        }

        if (cmd.rArm.t[i] != cmd.rArm.t[i]) {
            return false;
        }
    }

    // TODO: check FRI commands
//    cmd.lArm.cmd.data = static_cast<int32_t >(rand());
//    cmd.lArm.cmd_valid = true;
//    cmd.rArm.cmd.data = static_cast<int32_t >(rand());
//    cmd.rArm.cmd_valid = true;

    if (cmd.lHand.q.size() != 4) {
        return false;
    }

    if (cmd.lHand.dq.size() != 4) {
        return false;
    }

    if (cmd.lHand.max_p.size() != 4) {
        return false;
    }

    if (cmd.lHand.max_i.size() != 4) {
        return false;
    }

    if (cmd.rHand.q.size() != 4) {
        return false;
    }

    if (cmd.rHand.dq.size() != 4) {
        return false;
    }

    if (cmd.rHand.max_p.size() != 4) {
        return false;
    }

    if (cmd.rHand.max_i.size() != 4) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (cmd.lHand.q[i] != cmd.lHand.q[i]) {
            return false;
        }

        if (cmd.lHand.dq[i] != cmd.lHand.dq[i]) {
            return false;
        }

        if (cmd.lHand.max_p[i] != cmd.lHand.max_p[i]) {
            return false;
        }

        if (cmd.lHand.max_i[i] != cmd.lHand.max_i[i]) {
            return false;
        }

        if (cmd.rHand.q[i] != cmd.rHand.q[i]) {
            return false;
        }

        if (cmd.rHand.dq[i] != cmd.rHand.dq[i]) {
            return false;
        }

        if (cmd.rHand.max_p[i] != cmd.rHand.max_p[i]) {
            return false;
        }

        if (cmd.rHand.max_i[i] != cmd.rHand.max_i[i]) {
            return false;
        }
    }

//    cmd.lHand.hold = static_cast<bool >(rand()%2);
//    cmd.rHand.hold = static_cast<bool >(rand()%2);
//    cmd.lHand_valid = true;
//    cmd.rHand_valid = true;

    // TODO: check ranges

    return true;
}

void VelmaLowLevelSafety::updateHook() {
    Logger::In in("VelmaLowLevelSafety::updateHook");
//    RESTRICT_ALLOC;

    no_hw_error_counter_++;

    bool status_valid = false;
    if (port_status_in_.read(status_in_) == NewData) {


        tFriIntfState *lArmFri = reinterpret_cast<tFriIntfState*>(&status_in_.lArm.friIntfState[0]);
        tFriRobotState *lArmRobot = reinterpret_cast<tFriRobotState*>(&status_in_.lArm.friRobotState[0]);

        tFriIntfState *rArmFri = reinterpret_cast<tFriIntfState*>(&status_in_.rArm.friIntfState[0]);
        tFriRobotState *rArmRobot = reinterpret_cast<tFriRobotState*>(&status_in_.rArm.friRobotState[0]);

/*
    for (int i = 0; i < 8; ++i) {
        status.lHand.q[i] = static_cast<double >(rand());
        status.rHand.q[i] = static_cast<double >(rand());
    }
    status.lHand.s = static_cast<uint32_t >(rand());
    status.rHand.s = static_cast<uint32_t >(rand());

    for (int i = 0; i < 7; ++i) {
        status.lArm.q[i] = static_cast<double >(rand());
        status.lArm.dq[i] = static_cast<double >(rand());
        status.lArm.t[i] = static_cast<double >(rand());
        status.lArm.gt[i] = static_cast<double >(rand());
        status.rArm.q[i] = static_cast<double >(rand());
        status.rArm.dq[i] = static_cast<double >(rand());
        status.rArm.t[i] = static_cast<double >(rand());
        status.rArm.gt[i] = static_cast<double >(rand());
    }

    for (int i = 0; i < 28; ++i) {
        status.lArm.mmx[i] = static_cast<double >(rand());
        status.rArm.mmx[i] = static_cast<double >(rand());
    }

    status.lArm.w.force.x = static_cast<double >(rand());
    status.lArm.w.force.y = static_cast<double >(rand());
    status.lArm.w.force.z = static_cast<double >(rand());
    status.lArm.w.torque.x = static_cast<double >(rand());
    status.lArm.w.torque.y = static_cast<double >(rand());
    status.lArm.w.torque.z = static_cast<double >(rand());

    status.tMotor_q = static_cast<double >(rand());
    status.tMotor_dq = static_cast<double >(rand());
    status.hpMotor_q = static_cast<double >(rand());
    status.hpMotor_dq = static_cast<double >(rand());
    status.htMotor_q = static_cast<double >(rand());
    status.htMotor_dq = static_cast<double >(rand());

    for (int i = 0; i < 24; ++i) {
        status.rHand_p.finger1_tip[i] = static_cast<int16_t >(rand());
        status.rHand_p.finger2_tip[i] = static_cast<int16_t >(rand());
        status.rHand_p.finger3_tip[i] = static_cast<int16_t >(rand());
        status.rHand_p.palm_tip[i] = static_cast<int16_t >(rand());
    }

    for (int i = 0; i < 3; ++i) {
        status.lHand_f[i].wrench.force.x = static_cast<double >(rand());
        status.lHand_f[i].wrench.force.y = static_cast<double >(rand());
        status.lHand_f[i].wrench.force.z = static_cast<double >(rand());
        status.lHand_f[i].wrench.torque.x = static_cast<double >(rand());
        status.lHand_f[i].wrench.torque.y = static_cast<double >(rand());
        status.lHand_f[i].wrench.torque.z = static_cast<double >(rand());
    }

    status.rFt.rw.force.x = static_cast<double >(rand());
    status.rFt.rw.force.y = static_cast<double >(rand());
    status.rFt.rw.force.z = static_cast<double >(rand());
    status.rFt.rw.torque.x = static_cast<double >(rand());
    status.rFt.rw.torque.y = static_cast<double >(rand());
    status.rFt.rw.torque.z = static_cast<double >(rand());

    status.lFt.rw.force.x = static_cast<double >(rand());
    status.lFt.rw.force.y = static_cast<double >(rand());
    status.lFt.rw.force.z = static_cast<double >(rand());
    status.lFt.rw.torque.x = static_cast<double >(rand());
    status.lFt.rw.torque.y = static_cast<double >(rand());
    status.lFt.rw.torque.z = static_cast<double >(rand());
*/


        if (lArmRobot->power != 0x7F || rArmRobot->power != 0x7F || lArmRobot->error != 0 || rArmRobot->error != 0 ||   // error
            lArmRobot->warning != 0 || rArmRobot->warning != 0 ||   // error?
            lArmRobot->control != FRI_CTRL_JNT_IMP || rArmRobot->control != FRI_CTRL_JNT_IMP ||
            lArmFri->quality <= FRI_QUALITY_UNACCEPTABLE || rArmFri->quality <= FRI_QUALITY_UNACCEPTABLE)
        {
            // error
            no_hw_error_counter_ = 0;
        }

        lArm_fri_state_ = lArmFri->state;
        rArm_fri_state_ = rArmFri->state;

        // robot power
        if (lArmRobot->power != 0x7F) {
            // error
            std::cout << "lArmRobot->power != 0x7F" << std::endl;
        }
        else if (rArmRobot->power != 0x7F) {
            // error
            std::cout << "rArmRobot->power != 0x7F" << std::endl;
        }
        else if (lArmRobot->error != 0) {
            // error
            std::cout << "lArmRobot->error != 0" << std::endl;
        }
        else if (rArmRobot->error != 0) {
            // error
            std::cout << "rArmRobot->error != 0" << std::endl;
        }
        else if (lArmRobot->warning != 0) {
            // error?
            std::cout << "lArmRobot->warning" << std::endl;
        }
        else if (rArmRobot->warning != 0) {
            // error?
            std::cout << "rArmRobot->warning" << std::endl;
        }
        else if (lArmRobot->control != FRI_CTRL_JNT_IMP) {
            // error
            std::cout << "lArmRobot->control != FRI_CTRL_JNT_IMP" << std::endl;
        }
        else if (rArmRobot->control != FRI_CTRL_JNT_IMP) {
            // error
            std::cout << "rArmRobot->control != FRI_CTRL_JNT_IMP" << std::endl;
        }
        // FRI link quality
        else if (lArmFri->quality <= FRI_QUALITY_UNACCEPTABLE) {
            // error
        }
        else if (rArmFri->quality <= FRI_QUALITY_UNACCEPTABLE) {
            // error
        }
/*
        // FRI state
        if (lArmFri->state == FRI_STATE_MON) {      // FRI_STATE_CMD
        }
*/



/*
        if (port_KRL_CMD_in_.read(KRL_CMD_in_) == RTT::NewData) {
            if (KRL_CMD_in_.data == 1) {
                command_mode_ = true;
            }
            else if (KRL_CMD_in_.data == 2) {
                command_mode_ = false;
            }
        }

*/

        // TODO: verify robot status here
        status_valid = true;
    }
    else {
        // could not receive valid status data
        std::cout << "could not receive valid status data" << std::endl;
        no_hw_error_counter_ = 0;
    }

    uint32_t comm_status_in = 0;
    if (port_command_in_.read(cmd_in_) == NewData && is_command_valid(cmd_in_)) {
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

        if (enable_command_mode_switch_ && lArm_fri_state_ == FRI_STATE_MON && no_hw_error_counter_ > 1000) {
            cmd_out_.lArm.cmd.data = 1;
            cmd_out_.lArm.cmd_valid = true;
            Logger::log() << Logger::Info << "switching lArm to command mode..." << Logger::endl;
        }
        else {
            cmd_out_.lArm.cmd_valid = false;
        }

        if (enable_command_mode_switch_ && rArm_fri_state_ == FRI_STATE_MON && no_hw_error_counter_ > 1000) {
            cmd_out_.rArm.cmd.data = 1;
            cmd_out_.rArm.cmd_valid = true;
            Logger::log() << Logger::Info << "switching rArm to command mode..." << Logger::endl;
        }
        else {
            cmd_out_.rArm.cmd_valid = false;
        }

        if (status_valid) {
            // right arm
//            arm_q_.convertFromROS(status_in_.rArm.q);
//            arm_dq_.convertFromROS(status_in_.rArm.dq);
//            arm_mass77_.convertFromROS(status_in_.rArm.mmx);
//            if ( calculateJntImpTorque(rArm_safe_q_.data_, arm_q_.data_, arm_dq_.data_,
//                arm_k_, arm_mass77_.data_, arm_t_cmd_.data_) )
//            {
//                arm_t_cmd_.convertToROS(cmd_out_.rArm.t);
//            }

            arm_dq_.convertFromROS(status_in_.rArm.dq);
            calculateArmDampingTorque(arm_dq_.data_, arm_t_cmd_.data_);
            arm_t_cmd_.convertToROS(cmd_out_.rArm.t);

            // left arm
//            arm_q_.convertFromROS(status_in_.lArm.q);
//            arm_dq_.convertFromROS(status_in_.lArm.dq);
//            arm_mass77_.convertFromROS(status_in_.lArm.mmx);
//            if ( calculateJntImpTorque(lArm_safe_q_.data_, arm_q_.data_, arm_dq_.data_,
//                arm_k_, arm_mass77_.data_, arm_t_cmd_.data_) )
//            {
//                arm_t_cmd_.convertToROS(cmd_out_.lArm.t);
//            }

            arm_dq_.convertFromROS(status_in_.lArm.dq);
            calculateArmDampingTorque(arm_dq_.data_, arm_t_cmd_.data_);
            arm_t_cmd_.convertToROS(cmd_out_.lArm.t);
        }

        out_.writePorts(cmd_out_);
    }
}

ORO_LIST_COMPONENT_TYPE(VelmaLowLevelSafety)
ORO_CREATE_COMPONENT_LIBRARY();

