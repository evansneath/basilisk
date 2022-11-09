/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef PRESCRIBED_MOTION_STATE_EFFECTOR_H
#define PRESCRIBED_MOTION_STATE_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/PrescribedMotionMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief prescribed motion state effector class */
class PrescribedMotionStateEffector: public StateEffector, public SysModel {
public:
    // GIVEN QUANTITIES FROM USER PYTHON INTERFACE
    double mass;
    Eigen::Matrix3d IHubBc_B;
    Eigen::Matrix3d IPntFc_F;
    Eigen::Vector3d r_MB_B;
    Eigen::Vector3d r_FcF_F;
    Eigen::Vector3d r_FM_MInit;
    Eigen::Vector3d rPrime_FM_MInit;
    Eigen::Vector3d rPrimePrime_FM_MInit;
    Eigen::Vector3d omega_BN_BInit;
    double theta_FBInit;
    double thetaDot_FBInit;
    Eigen::Matrix3d dcm_F0B;
    int rotAxisNum;
    double u_B;

    Message<PrescribedMotionMsgPayload> prescribedMotionOutMsg;
    Message<SCStatesMsgPayload> prescribedMotionConfigLogOutMsg;
    BSKLogger bskLogger;

private:
    static uint64_t effectorID;

    // GIVEN QUANTITIES FROM USER PYTHON INTERFACE IN BODY FRAME
    Eigen::Matrix3d IPntFc_B; // OR DEFINE WITHIN MEMBER FUNCTION     Eigen::Vector3d IPntFc_B = this->dcmBF*this->IPntFc_F
    Eigen::Vector3d r_FB_B; // DEFINE WITHIN MEMBER FUNCTION     Eigen::Vector3d r_FB_B = r_FM_B + this->r_MB_B
    Eigen::Vector3d r_FcF_B; // OR DEFINE WITHIN MEMBER FUNCTION     Eigen::Vector3d r_FcF_B = this->dcmBF*this->r_FcF_F

    // PRESCRIBED PARAMETERS
    Eigen::Vector3d r_FM_M;
    Eigen::Vector3d rPrime_FM_M;
    Eigen::Vector3d rPrimePrime_FM_M;
    Eigen::Vector3d omega_FB_F;
    Eigen::Vector3d omegaPrime_FB_F;

    // PRESCRIBED PARAMETERS IN BODY FRAME
    Eigen::Vector3d r_FM_B;
    Eigen::Vector3d rPrime_FM_B;
    Eigen::Vector3d rPrimePrime_FM_B;
    Eigen::Vector3d omega_FB_B;
    Eigen::Vector3d omegaPrime_FB_B;

    // Other vector quantities
    Eigen::Vector3d r_FcB_B;
    Eigen::Vector3d r_FcM_B;
    Eigen::Vector3d rPrime_FcM_B;
    Eigen::Vector3d rPrime_FcB_B;
    Eigen::Vector3d rPrimePrime_FcB_B;
    Eigen::Vector3d omega_BN_B;
    Eigen::Vector3d omega_FN_B;
    Eigen::Vector3d rDot_FcB_B;
    Eigen::MRPd sigma_BN;
    Eigen::Vector3d sigma_FB;

    // DCMs
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_BF;
    Eigen::Matrix3d dcm_BM;

    // Other matrix quantities
    Eigen::Matrix3d rTilde_FcB_B;
    Eigen::Matrix3d omegaTilde_BN_B;
    Eigen::Matrix3d omegaTilde_FB_B;

    // Effector body properties
    Eigen::Vector3d r_FcN_N;
    Eigen::Vector3d v_FcN_N;
    Eigen::Vector3d sigma_FN;
    Eigen::Vector3d omega_FN_F;

    // States
    StateData *hubSigma;
    StateData *hubOmega;
    StateData *hubPosition;
    StateData *hubVelocity;
    Eigen::MatrixXd *c_B;
    Eigen::MatrixXd *cPrime_B;

public:
    PrescribedMotionStateEffector();    //!< -- Constructor
    ~PrescribedMotionStateEffector();   //!< -- Destructor
    void Reset(uint64_t CurrentClock);                   //!< -- Method for reset
    void writeOutputStateMessages(uint64_t CurrentClock);   //!< -- Method for writing the output messages
	void UpdateState(uint64_t CurrentSimNanos);             //!< -- Method for updating information
    void registerStates(DynParamManager& statesIn);         //!< -- Method for registering the effector's states
    void linkInStates(DynParamManager& states);             //!< -- Method for getting access to other states
    void updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N);  //!< -- Method for back-substitution contributions
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);                         //!< -- Method for effector to compute its derivatives
    void updateEffectorMassProps(double integTime);         //!< -- Method for giving the s/c the HRB mass props and prop rates
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B);       //!< -- Method for computing energy and momentum for effectors
    void prependSpacecraftNameToStates();                   //!< Method used for multiple spacecraft
    void computePrescribedMotionInertialStates();               //!< Method for computing the effector's states
    void computePrescribedParameters(double integTime);

};

#endif /* PRESCRIBED_MOTION_STATE_EFFECTOR_H */
