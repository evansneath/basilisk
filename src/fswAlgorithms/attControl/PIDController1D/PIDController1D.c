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


/* modify the path to reflect the new module names */
#include "PIDController1D.h"
#include "string.h"
#include <math.h>

/* Support files.  Be sure to use the absolute path relative to Basilisk directory. */
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"


/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_PIDController1D(PIDController1DConfig *configData, int64_t moduleID)
{
    ArrayMotorTorqueMsg_C_init(&configData->motorTorqueOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_PIDController1D(PIDController1DConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required input message is included
    if (!SpinningBodyMsg_C_isLinked(&configData->spinningBodyInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.spinningBodyInMsg wasn't connected.");
    }
    // check if the required input message is included
    if (!SpinningBodyMsg_C_isLinked(&configData->spinningBodyRefInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: solarArrayAngle.spinningBodyRefInMsg wasn't connected.");
    }
}

/*! This method computes the control torque to the solar array drive based on a PD control law
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_PIDController1D(PIDController1DConfig *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Create buffer messages */
    SpinningBodyMsgPayload      spinningBodyIn;
    SpinningBodyMsgPayload      spinningBodyRefIn;
    ArrayMotorTorqueMsgPayload  motorTorqueOut;

    /*! - zero the output message */
    motorTorqueOut = ArrayMotorTorqueMsg_C_zeroMsgPayload();

    /*! read the solar array angle message */
    spinningBodyIn = SpinningBodyMsg_C_read(&configData->spinningBodyInMsg);

    /*! read the solar array reference angle message */
    spinningBodyRefIn = SpinningBodyMsg_C_read(&configData->spinningBodyRefInMsg);

    /* extract angles and angle rates */
    double thetaR, thetaC, thetaDotR, thetaDotC;
    thetaR    = spinningBodyRefIn.theta;
    thetaDotR = spinningBodyRefIn.thetaDot;
    thetaC    = spinningBodyIn.theta;
    thetaDotC = spinningBodyIn.thetaDot;

    /* extract gains from input */
    double K, P;
    K = configData->K;
    P = configData->P;

    /* compute torque */
    double T = K * (thetaR - thetaC) + P * (thetaDotR - thetaDotC);
    motorTorqueOut.motorTorque[0] = T;

    /* write output message */
    ArrayMotorTorqueMsg_C_write(&motorTorqueOut, &configData->motorTorqueOutMsg, moduleID, callTime);

    return;
}