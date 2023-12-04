# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Json Zhou 
# =============================================================================
#
# Demo to show Unitree Go1 operated on Rigid Terrain
# =============================================================================

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
try:
   from pychrono import irrlicht as chronoirr
except:
   print('Could not import ChronoIrrlicht')

# Chreate Chrono system
system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChMaterialSurfaceSMC()
ground_mat.SetFriction(0.9)
ground_mat.SetYoungModulus(1e7)
ground = chrono.ChBodyEasyBox(20, 20, 0.1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Unitree robot
unitree = robot.Unitree_Go1(system)
unitree.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, 0.0, 0.50), chrono.ChQuaternionD(1, 0, 0, 0)))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2.5, 1.5), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

####vis.EnableShadows()

time_step = 5e-4

# Simulation loop
time = 0
step = 1
while (vis.Run()) :
    time = time + time_step
            
    if(step % 100 == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        step = 1
    else:
        step+=1
        
    system.DoStepDynamics(time_step)
