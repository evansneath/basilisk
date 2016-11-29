''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus() and gravity modules.  Illustrates
#           a 3-DOV spacecraft on a range of orbit types.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 26, 2016
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

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import gravityEffector
import simIncludeGravity







# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("orbitCase, useSphericalHarmonics, planetCase", [
      (0, False,0)
    , (1, False,0)
    , (2, False,0)
    , (0, True, 0)
    , (0, False,1)
])

# provide a unique test method name, starting with test_
def test_scenarioBasicOrbit(show_plots, orbitCase, useSphericalHarmonics, planetCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, orbitCase, useSphericalHarmonics, planetCase)
    assert testResults < 1, testMessage



## This scenario demonstrates how to setup basic 3-DOF orbits.
#
# Basic Orbit Setup and Translational Motion SImulation {#scenarioBasicOrbit}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is orbiting a planet.  The purpose
# is to illustrate how to create a spacecraft, attach a gravity model, and run
# a basic Basilisk simulation.  The scenarios can be run with the followings setups
# parameters:
# Setup | orbitCase           | useSphericalHarmonics | planetCase
# ----- | ------------------- | --------------------- | -----------
# 1     | 0 (LEO)             | False                 | 0 (Earth)
# 2     | 1 (GTO)             | False                 | 0 (Earth)
# 3     | 2 (GEO)             | False                 | 0 (Earth)
# 4     | 0 (LEO)             | True                  | 0 (Earth)
# 5     | 0 (LMO)             | False                 | 1 (Mars)
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioBasicOrbit.py
#
# When the simulation completes 2 plots are shown for each case.  One plot always shows
# the inertial position vector components, while the second plot either shows a planar
# orbit view relative to the perfocal frame (no spherical harmonics), or the
# semi-major axis time history plot (with spherical harmonics turned on).
#
# The dynamics simulation is setup using a SpacecraftPlus() module.  Note that the rotational motion simulation is turned off to leave
# pure 3-DOF translation motion simulation.
#~~~~~~~~~~~~~~~~~{.py}
#     scObject = spacecraftPlus.SpacecraftPlus()
#     scObject.ModelTag = "spacecraftBody"
#     scObject.hub.useTranslation = True
#     scObject.hub.useRotation = False
#~~~~~~~~~~~~~~~~~
# Next, this module is attached to the simulation process
#~~~~~~~~~~~~~~~~~{.py}
#   scSim.AddModelToTask(simTaskName, scObject)
#~~~~~~~~~~~~~~~~~
# To attach an Earth gravity model to this spacecraft, the following macro is invoked:
#~~~~~~~~~~~~~~~~~{.py}
#     gravBody, ephemData = simIncludeGravity.addEarth()
#     gravBody.isCentralBody = True          # ensure this is the central gravitational body
#~~~~~~~~~~~~~~~~~
# If extra customization is required, see teh addEarth() macro to change additional values.
# For example, the spherical harmonics are turned off by default.  To engage them, the following code
# is used
#~~~~~~~~~~~~~~~~~{.py}
#     gravBody.useSphericalHarmParams = True
#     gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/SphericalHarmonics/Earth_GGM03S.txt'
#                                      , gravBody.spherHarm
#                                      ,3
#                                      )
#~~~~~~~~~~~~~~~~~
# The value 3 indidates that the first three harmonics, including the 0th order harmonic,
# is included.
#
#
# Setup 1
# -----
#
# Which scenario is run is controlled at the bottom of the file in the code
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          0,           # orbit Case
#          False,       # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# The first 2 arguments can be left as is.  The last 2 arguments control the
# simulation scenario flags to turn on or off certain simulation conditions.  The default
# scenario places the spacecraft about the Earth in a LEO orbit and without considering
# gravitational spherical harmonics.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1000.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2000.svg "Orbit Illustration")
#
# Setup 2
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          1,           # orbit Case
#          False,       # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates an elliptical Geosynchronous Transfer Orbit (GTO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1100.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2100.svg "Orbit Illustration")
#
# Setup 3
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          2,           # orbit Case
#          False,       # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Geosynchronous Orbit (GEO) with zero orbit
# inclination.  The
# resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1200.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2200.svg "Orbit Illustration")
#
#  Setup 4
# -----
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          0,           # orbit Case
#          True,        # useSphericalHarmonics
#          0            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular LEO with a non-zero orbit
# inclination.  In this case the Earth's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1010.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2010.svg "Orbit Illustration")
#
# Setup 5
# -------
#
# The next scenario is run by changing the bottom of the file in the scenario code to read
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run( False,       # do unit tests
#          True,        # show_plots
#          0,           # orbit Case
#          True,        # useSphericalHarmonics
#          1            # planet Case
#        )
# ~~~~~~~~~~~~~
# This case illustrates a circular Low Mars Orbit or LMO with a non-zero orbit
# inclination.  In this case the Earth's spherical harmonics are turned on.  The
# resulting position coordinates and semi-major axis time histories are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioBasicOrbit1001.svg "Position history")
# ![Perifocal Orbit Illustration](Images/Scenarios/scenarioBasicOrbit2001.svg "Orbit Illustration")
#

def run(doUnitTests, show_plots, orbitCase, useSphericalHarmonics, planetCase):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))


    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)


    # setup Gravity Body
    if planetCase == 1:     # Mars
        gravBody, ephemData = simIncludeGravity.addMars()
        gravBody.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            gravBody.useSphericalHarmParams = True
            gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM2BData.txt'
                                             , gravBody.spherHarm
                                             ,3
                                             )
        mu = gravBody.mu
    else:                   # Earth
        gravBody, ephemData = simIncludeGravity.addEarth()
        gravBody.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            gravBody.useSphericalHarmParams = True
            gravityEffector.loadGravFromFile(splitPath[0]+'/Basilisk/External/LocalGravData/GGM03S.txt'
                                             , gravBody.spherHarm
                                             ,3
                                             )
        mu = gravBody.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([gravBody])

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.*1000;      # meters
    rGEO = 42000.*1000;     # meters
    if orbitCase == 2:      # GTO case
        oe.a     = rGEO
        oe.e     = 0.00001
        oe.i     = 0.0*macros.D2R
    elif orbitCase == 1:    # GEO case
        oe.a = (rLEO+rGEO)/2.0
        oe.e = 1.0 - rLEO/oe.a
        oe.i = 0.0*macros.D2R
    else:                   # LEO case, default case 0
        oe.a     = rLEO
        oe.e     = 0.0001
        oe.i     = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f     = 85.3*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n
    if useSphericalHarmonics:
        simulationTime = macros.sec2nano(3.*P)
    else:
        simulationTime = macros.sec2nano(0.75*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    if useSphericalHarmonics:
        numDataPoints = 400
    else:
        numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)


    #
    # create simulation messages
    #

    # create the gravity ephemerise message
    scSim.TotalSim.CreateNewMessage(simProcessName,
                                    gravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 2)
    scSim.TotalSim.WriteMessageData(gravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 0,
                                          ephemData)


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    #
    #   initialize Spacecraft States within the state manager
    #   this must occur after the initialization
    #
    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    posRef.setState(unitTestSupport.np2EigenVector3d(rN))  # m - r_BN_N
    velRef.setState(unitTestSupport.np2EigenVector3d(vN))  # m - v_BN_N


    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N',range(3))

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]

    # draw the inertial position vector components
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(1,4):
        plt.plot(posData[:, 0]*macros.NANO2SEC/P, posData[:, idx]/1000.,
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$r_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"1"+str(int(orbitCase))+str(int(useSphericalHarmonics))
            +str(int(planetCase))
            , plt, path)

    if useSphericalHarmonics == False:
        # draw orbit in perifocal frame
        oeData = orbitalMotion.rv2elem(mu,posData[0,1:4],velData[0,1:4])
        b = oeData.a*np.sqrt(1-oeData.e*oeData.e)
        p = oeData.a*(1-oeData.e*oeData.e)
        plt.figure(2,figsize=np.array((1.0, b/oeData.a))*4.75,dpi=100)
        plt.axis(np.array([-oeData.rApoap, oeData.rPeriap, -b, b])/1000*1.25)
        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        if planetCase == 1:
            planetColor = '#884400'
        else:
            planetColor= '#008800'
        planetRadius = gravBody.radEquator/1000
        ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
        # draw the actual orbit
        rData=[]
        fData=[]
        for idx in range(0,len(posData)):
            oeData = orbitalMotion.rv2elem(mu,posData[idx,1:4],velData[idx,1:4])
            rData.append(oeData.rmag)
            fData.append(oeData.f)
        plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
                 ,color='#aa0000'
                 ,linewidth = 3.0
                 )
        # draw the full osculating orbit from the initial conditions
        fData = np.linspace(0,2*np.pi,100)
        rData = []
        for idx in range(0,len(fData)):
            rData.append(p/(1+oeData.e*np.cos(fData[idx])))
        plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
                 ,'--'
                 , color='#555555'
                 )
        plt.xlabel('$i_e$ Cord. [km]')
        plt.ylabel('$i_p$ Cord. [km]')
        plt.grid()
        if doUnitTests:     # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(
                fileNameString+"2"+str(int(orbitCase))+str(int(useSphericalHarmonics))
                +str(int(planetCase))
                , plt, path)
    else:
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        smaData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
            smaData.append(oeData.a/1000.)
        plt.plot(posData[:, 0]*macros.NANO2SEC/P, smaData
                 ,color='#aa0000',
                 )
        plt.xlabel('Time [orbits]')
        plt.ylabel('SMA [km]')
        if doUnitTests:     # only save off the figure if doing a unit test run
            unitTestSupport.saveScenarioFigure(
                fileNameString+"2"+str(int(orbitCase))+str(int(useSphericalHarmonics))
                +str(int(planetCase))
                , plt, path)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    #
    if doUnitTests:
        numTruthPoints = 5
        skipValue = int(numDataPoints/numTruthPoints)
        dataPosRed = posData[::skipValue]

        # setup truth data for unit test
        if orbitCase == 0 and useSphericalHarmonics == False and planetCase == 0:
            truePos = [
                  [-2.8168016010234905e+06, 5.2481748469161475e+06, 3.6771572646772973e+06]
                , [-6.3832193594279224e+06,-9.1678071954736591e+05, 2.7243803573345565e+06]
                , [-3.2242072294495562e+06,-6.1159997531577712e+06,-1.0989183586217165e+06]
                , [ 3.3316899420044743e+06,-4.8713265646545375e+06,-3.7642954993209662e+06]
                , [ 6.3762384860987831e+06, 1.5066729924376707e+06,-2.4626893874916704e+06]
            ]
        if orbitCase == 1 and useSphericalHarmonics == False and planetCase == 0:
            truePos = [
                  [-5.8895298480664780e+06, 9.6865748900076691e+06, 0.0000000000000000e+00]
                , [-2.9841899868063834e+07,-1.5137720479948777e+06, 0.0000000000000000e+00]
                , [-3.6242114305386089e+07,-1.4587361330633366e+07, 0.0000000000000000e+00]
                , [-3.4141961579096034e+07,-2.4456639423504535e+07, 0.0000000000000000e+00]
                , [-2.5748797606003813e+07,-2.9822602085271571e+07, 0.0000000000000000e+00]
            ]
        if orbitCase == 2 and useSphericalHarmonics == False and planetCase == 0:
            truePos = [
                  [-2.1819784817951124e+07, 3.5887241456518754e+07, 0.0000000000000000e+00]
                , [-4.1894573568364032e+07, 2.9785407713101692e+06, 0.0000000000000000e+00]
                , [-2.6678165220398702e+07,-3.2439322536990236e+07, 0.0000000000000000e+00]
                , [ 1.1011283367405793e+07,-4.0531027214480065e+07, 0.0000000000000000e+00]
                , [ 3.9424800586523451e+07,-1.4479829266943770e+07, 0.0000000000000000e+00]
            ]
        if orbitCase == 0 and useSphericalHarmonics == True and planetCase == 0:
            truePos = [
                  [-2.8168016010234905e+06, 5.2481748469161475e+06, 3.6771572646772973e+06]
                , [ 6.3768027982985154e+06, 1.5301978565714115e+06,-2.4340171199729838e+06]
                , [-2.1234793124024896e+06,-6.4156882591431364e+06,-1.8098650417713353e+06]
                , [-4.7524455240083179e+06, 3.4339560991056389e+06, 3.8240860083611421e+06]
                , [ 5.7958244639578946e+06, 3.7597384950639764e+06,-1.1050131437256800e+06]
            ]
        if orbitCase == 0 and useSphericalHarmonics == False and planetCase == 1:
            truePos = [
                  [-2.8168016010234905e+06, 5.2481748469161475e+06, 3.6771572646772973e+06]
                , [-6.3377484427205361e+06,-3.4255546782816760e+05, 2.9535269173761583e+06]
                , [-4.1455299024167331e+06,-5.6246507782849586e+06,-4.3263203566448629e+05]
                , [ 1.7834756999182550e+06,-5.8365487146244226e+06,-3.4287595557745439e+06]
                , [ 6.1043005786560886e+06,-7.8646594985982473e+05,-3.3335301636885651e+06]
            ]

        # compare the results to the truth values
        accuracy = 1e-6

        testFailCount, testMessages = unitTestSupport.compareArray(
            truePos, dataPosRed, accuracy, "r_BN_N Vector",
            testFailCount, testMessages)

        #   print out success message if no error were found
        if testFailCount == 0:
            print "PASSED "
        else:
            print testFailCount
            print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run( False,       # do unit tests
         True,        # show_plots
         0,           # orbit Case (0 - LEO, 1 - GTO, 2 - GEO)
         True,       # useSphericalHarmonics
         0            # planetCase (0 - Earth, 1 - Mars)
       )
