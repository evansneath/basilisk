'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

'''
#
#   Integrated Unit Test Script
#   Purpose:  Run a test of the IMU sensor module
#   Author:  John Alcorn
#   Creation Date:  September 6, 2016
#

import pytest
import sys, os, inspect
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import ctypes
import math
import csv
import logging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('basilisk')
sys.path.append(splitPath[0]+'/basilisk/modules')
sys.path.append(splitPath[0]+'/basilisk/PythonModules')

import MessagingAccess
import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import macros
import imu_sensor
import six_dof_eom
import sim_model
import RigidBodyKinematics as rbk

np.random.seed(200000)

# methods
def listStack(vec,simStopTime,unitProcRate):
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

def randpm1(n):
    return np.random.rand(n)*2 - 1

def nextMRP(sigma,omega,dt):
    # 1/4 [B(Q)] w
    dsigma = 0.25*np.dot(rbk.BmatMRP(sigma),np.reshape(omega,[3,1]))
    return dsigma*dt + np.reshape(sigma,[3,1])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

testNames = ['mrp switch','bias','noise','discretization','saturation','misalignment','CoM offset','walk bounds']

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, testCase", [
    (False,'mrp switch'),
    (False,'bias'),
    (False,'noise'),
    (False,'discretization'),
    pytest.mark.xfail((False,'saturation')),
    (False,'misalignment'),
    (False,'CoM offset'),
])

# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots, useFlag, testCase):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots, useFlag, testCase):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    # initialize SimulationBaseClass
    unitSim = SimulationBaseClass.SimBaseClass()
    unitSim.TotalSim.terminateSimulation()

    # create the task and specify the integration update time
    unitProcRate = macros.sec2nano(0.01)
    unitProcRate_s = macros.NANO2SEC*unitProcRate
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitProc.addTask(unitSim.CreateNewTask(unitTaskName, unitProcRate))


    # configure module
    ImuSensor = imu_sensor.ImuSensor()
    ImuSensor.ModelTag = "imusensor"
    ImuSensor.SensorPosStr = imu_sensor.DoubleVector([0.0, 0.0, 0.0])
    ImuSensor.setStructureToPlatformDCM(0.0, 0.0, 0.0)
    ImuSensor.accelLSB = 0.0 # 2.77E-4 * 9.80665
    ImuSensor.gyroLSB = 0.0 # 8.75E-3 * math.pi / 180.0
    ImuSensor.senRotBias = [0.0] * 3
    ImuSensor.senRotNoiseStd = [0.0] * 3 # 0.000001
    ImuSensor.senTransBias = [0.0] * 3
    ImuSensor.senTransNoiseStd = [0.0] * 3 # 1.0E-6
    ImuSensor.senRotMax = 1.0e6
    ImuSensor.senTransMax = 1.0e6

    # configure MassPropsData
    MassPropsData = six_dof_eom.MassPropsData()
    MassPropsData.Mass = -97.9
    MassPropsData.CoM = [0,0,0]
    MassPropsData.InertiaTensor = [-98.9] * 9
    MassPropsData.T_str2Bdy = [-99.9] * 9

    # configure module input message
    StateCurrent = six_dof_eom.OutputStateData()
    StateCurrent.r_N = [0,0,0]
    StateCurrent.v_N = [0,0,0]
    StateCurrent.sigma = np.array([0,0,0])
    StateCurrent.omega = [0,0,0]
    # StateCurrent.T_str2Bdy = [[1,0,0],[0,1,0],[0,0,1]]
    # sim_model.doubleArray_setitem(StateCurrent.T_str2Bdy, 0, 1.0)
    StateCurrent.TotalAccumDVBdy = [0,0,0] # -9.66
    StateCurrent.MRPSwitchCount = 0

    # get module output fields
    ImuSensorOutput = imu_sensor.ImuSensorOutput()
    fieldNames = list()
    for fieldName in dir(ImuSensorOutput):
        if fieldName.find('__') < 0 and fieldName.find('this') < 0:
            fieldNames.append(fieldName)

    trueVector = dict()

    accel = [0.0,0.0,0.0]
    omega = [0.0,0.0,0.0]

    # configure tests
    if testCase == 'mrp switch':
        # done.
        # this test verifies basic input and output and checks the MRP switch
        simStopTime = 6.0 * 4 # run the sim long enough for the MRP to switch
        StateCurrent.sigma = np.array([0.9,0,0])
        omega = randpm1(3)*0.1
        StateCurrent.omega = omega
        accel = randpm1(3)*0.1
        trueVector['AngVelPlatform'] = listStack(omega,simStopTime,unitProcRate)
        trueVector['AccelPlatform'] = listStack(accel,simStopTime,unitProcRate)
        trueVector['DRFramePlatform'] = listStack(np.asarray(omega)*unitProcRate_s,simStopTime,unitProcRate)
        trueVector['DVFramePlatform'] = listStack(np.asarray(accel)*unitProcRate_s,simStopTime,unitProcRate)

    elif testCase == 'bias':
        # done.
        # this test verifies static bias
        simStopTime = 0.5
        senRotBias = randpm1(3)
        ImuSensor.senRotBias = senRotBias
        omega = randpm1(3)
        StateCurrent.omega = omega
        omegaOut = np.asarray(omega) + np.asarray(senRotBias) # sensor sees the sum of bias and truth
        accel = randpm1(3)
        senTransBias = randpm1(3)
        ImuSensor.senTransBias = senTransBias
        accelOut = np.asarray(accel) + np.asarray(senTransBias) # sensor sees the sum of bias and truth
        trueVector['AccelPlatform'] = listStack(accelOut,simStopTime,unitProcRate)
        trueVector['AngVelPlatform'] = listStack(omegaOut,simStopTime,unitProcRate)
        trueVector['DRFramePlatform'] = listStack(np.asarray(omegaOut)*unitProcRate_s,simStopTime,unitProcRate)
        trueVector['DVFramePlatform'] = listStack(np.asarray(accelOut)*unitProcRate_s,simStopTime,unitProcRate)

    elif testCase == 'noise':
        # done.
        # this test checks the standard deviation of sensor noise
        simStopTime = 1000.0
        senRotNoiseStd = np.random.rand(3)
        ImuSensor.senRotNoiseStd = senRotNoiseStd
        senTransNoiseStd = np.random.rand(3)
        ImuSensor.senTransNoiseStd = senTransNoiseStd
        trueVector['AccelPlatform'] = senTransNoiseStd
        trueVector['AngVelPlatform'] = senRotNoiseStd
        trueVector['DRFramePlatform'] = np.asarray(senRotNoiseStd)*unitProcRate_s
        trueVector['DVFramePlatform'] = np.asarray(senTransNoiseStd)*unitProcRate_s

    elif testCase == 'discretization':
        accuracy = 1e-5
        simStopTime = 0.5
        accelLSB = np.random.rand()*1e-4
        gyroLSB = np.random.rand()*1e-4
        ImuSensor.accelLSB = accelLSB # 2.77E-4 * 9.80665
        ImuSensor.gyroLSB = gyroLSB # 8.75E-3 * math.pi / 180.0
        omega = randpm1(3)*0.1 # [1.05,1.15,-1.29]
        StateCurrent.omega = omega
        accel = randpm1(3)*0.1 # [1.05,1.15,-1.29]
        omegaDiscretized = np.fix(np.asarray(omega)/gyroLSB)*gyroLSB
        # DR = -(omega - omegaDiscretized)*unitProcRate_s # why???? -john
        DR = omegaDiscretized*unitProcRate_s
        accelDiscretized = np.fix(np.asarray(accel)/accelLSB)*accelLSB
        DV = accelDiscretized*unitProcRate_s # why???? -john
        trueVector['AngVelPlatform'] = listStack(omegaDiscretized,simStopTime,unitProcRate)
        trueVector['AccelPlatform'] = listStack(accelDiscretized,simStopTime,unitProcRate)
        trueVector['DVFramePlatform'] = listStack(DV,simStopTime,unitProcRate)
        trueVector['DRFramePlatform'] = listStack(DR,simStopTime,unitProcRate)

    elif testCase == 'saturation':
        simStopTime = 0.1
        ImuSensor.senRotMax = .2
        ImuSensor.senTransMax = .3
        omega = [1.,-1.,0.123]
        StateCurrent.omega = omega
        omegaSaturated = [.2,-.2,0.123]
        accel = [2.,-0.213,-2.]
        accelSaturated = [.3,-0.213,-.3]
        trueVector['AngVelPlatform'] = listStack(omegaSaturated,simStopTime,unitProcRate)
        trueVector['AccelPlatform'] = listStack(accelSaturated,simStopTime,unitProcRate)
        trueVector['DVFramePlatform'] = listStack(np.asarray(accelSaturated)*unitProcRate_s,simStopTime,unitProcRate)
        trueVector['DRFramePlatform'] = listStack(np.asarray(omegaSaturated)*unitProcRate_s,simStopTime,unitProcRate)

    elif testCase == 'CoM offset':
        accuracy = 1e-5
        simStopTime = 0.5
        SensorPosStr = [0.21, -0.43, 0.65]
        ImuSensor.SensorPosStr = imu_sensor.DoubleVector(SensorPosStr)
        MassPropsData.CoM = [0,0,0]
        omega = np.array([1.2,3.4,-5.6]) * 0.1
        StateCurrent.omega = omega
        accelTrue = np.cross(np.asarray(omega),np.cross(np.asarray(omega),np.asarray(SensorPosStr)))
        trueVector['AngVelPlatform'] = listStack(omega,simStopTime,unitProcRate)
        trueVector['AccelPlatform'] = listStack(accelTrue,simStopTime,unitProcRate)
        trueVector['DVFramePlatform'] = listStack(np.asarray(accelTrue)*unitProcRate_s,simStopTime,unitProcRate)
        trueVector['DRFramePlatform'] = listStack(np.asarray(omega)*unitProcRate_s,simStopTime,unitProcRate)

    elif testCase == 'misalignment':
        accuracy = 1e-4
        simStopTime = 0.5
        euler = [0.6398,0.7114,0.0997]
        ImuSensor.setStructureToPlatformDCM(euler[0], euler[1], euler[2])
        omega = randpm1(3)
        StateCurrent.omega = omega
        omegaOut = np.dot(rbk.euler3212C(euler),np.asarray(omega))
        accel = randpm1(3)
        accelOut = np.dot(rbk.euler3212C(euler),np.asarray(accel))
        trueVector['AngVelPlatform'] = listStack(omegaOut,simStopTime,unitProcRate)
        trueVector['AccelPlatform'] = listStack(accelOut,simStopTime,unitProcRate)
        trueVector['DVFramePlatform'] = listStack(accelOut*unitProcRate_s,simStopTime,unitProcRate)
        trueVector['DRFramePlatform'] = listStack(omegaOut*unitProcRate_s,simStopTime,unitProcRate)

    elif testCase == 'walk bounds':
        simStopTime = 0.5
        omegaWalkBound = 6
        accelWalkBound = 6
        senRotNoiseStd = np.random.rand(3)
        ImuSensor.senRotNoiseStd = senRotNoiseStd
        senTransNoiseStd = np.random.rand(3)
        ImuSensor.senTransNoiseStd = senTransNoiseStd
        trueVector['AngVelPlatform'] = omegaWalkBound
        trueVector['AccelPlatform'] = accelWalkBound
        trueVector['DVFramePlatform'] = np.nan
        trueVector['DRFramePlatform'] = np.nan

    else:
        raise Exception('invalid test case')

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, ImuSensor)

    # log module output message
    unitSim.TotalSim.logThisMessage(ImuSensor.OutputDataMsg, unitProcRate)

    # configure spacecraft_mass_props message
    unitSim.TotalSim.CreateNewMessage("TestProcess", "spacecraft_mass_props", 8*3*4, 2)
    unitSim.TotalSim.WriteMessageData("spacecraft_mass_props", 8*3*4, 0, MassPropsData )

    # configure inertial_state_output message
    unitSim.TotalSim.CreateNewMessage("TestProcess", "inertial_state_output", 8*3*11, 2)
    unitSim.TotalSim.WriteMessageData("inertial_state_output", 8*3*11, 0, StateCurrent )

    unitSim.InitializeSimulation()


    if np.array_equal(accel,np.asarray([0.0,0.0,0.0])) and np.array_equal(omega,np.asarray([0.0,0.0,0.0])):
        simStopTimeStep = simStopTime
    else:
        simStopTimeStep = unitProcRate_s

    prevSimNanos = 0.
    velStep = np.asarray(accel)*unitProcRate_s
    for i in range(0,int(simStopTime/simStopTimeStep)+1):
        currSimNanos = unitSim.TotalSim.CurrentNanos
        dt = (currSimNanos - prevSimNanos) * macros.NANO2SEC
        prevSimNanos = currSimNanos
        StateCurrent.TotalAccumDVBdy = np.asarray(StateCurrent.TotalAccumDVBdy) + np.asarray(velStep)
        StateCurrent.sigma = nextMRP(StateCurrent.sigma,omega,dt)
        if np.dot(StateCurrent.sigma,StateCurrent.sigma) > 1.:
            StateCurrent.sigma = rbk.MRPswitch(np.asarray(StateCurrent.sigma),1.)
            StateCurrent.MRPSwitchCount += 1
        unitSim.TotalSim.WriteMessageData("inertial_state_output", 8*3*11, currSimNanos, StateCurrent )
        unitSim.ConfigureStopTime(macros.sec2nano(simStopTimeStep*float(i)))
        unitSim.ExecuteSimulation()

    # pull message log data and assemble into dict
    moduleOutput = dict()
    for moduleOutputName in fieldNames:
        moduleOutput[moduleOutputName] = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + moduleOutputName, range(3))
        print "\n\n" + moduleOutputName
        print str(moduleOutput[moduleOutputName]) + "\n"
        print str(trueVector[moduleOutputName]) + "\n\n"

    # plt.plot(moduleOutput['DRFramePlatform'][:,0]*macros.NANO2SEC,moduleOutput['DRFramePlatform'][:,1])
    # plt.show()

    # trim the truth and module output arrays
    if not testCase == 'noise':
        for moduleOutputName in fieldNames:
            trueVector[moduleOutputName] = np.asarray(trueVector[moduleOutputName][1:])
            newArr = np.delete(moduleOutput[moduleOutputName],(0),axis=0)
            del moduleOutput[moduleOutputName]
            moduleOutput[moduleOutputName] = newArr


    # compare the module results to the truth values
    if not 'accuracy' in vars():
        accuracy = 1e-3

    testFail = False
    for moduleOutputName in fieldNames:
        if testCase == 'noise':
            for i in range(0,3):
                if np.abs(np.mean(moduleOutput[moduleOutputName][:,i+1])) > 0.1 \
                                or np.abs(np.std(moduleOutput[moduleOutputName][:,i+1]) - trueVector[moduleOutputName][i]) > 0.1 :
                    testFail = True

        elif testCase == 'walk bounds':
            for i in range(0,3):
                if np.isnan(trueVector[moduleOutputName]):
                    continue
                elif np.max(np.abs(np.asarray(moduleOutput[moduleOutputName][:,i+1]))) > trueVector[moduleOutputName]:
                    testFail = True

        else:
            for i in range(0,len(trueVector[moduleOutputName])):
                if not unitTestSupport.isArrayEqual(moduleOutput[moduleOutputName][i], trueVector[moduleOutputName][i], 3, accuracy):
                    testFail = True

        if testFail:
            testFailCount += 1
            testMessages.append("FAILED: " + ImuSensor.ModelTag + " Module failed " +
                                moduleOutputName + " unit test at t=" +
                                str(moduleOutput[moduleOutputName][i,0]*macros.NANO2SEC) +
                                "sec\n")


    np.set_printoptions(precision=16)

    # print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED "

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unit test script can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitSimIMU(
        False, # show_plots
        False, # useFlag
        'saturation' # testCase
    )