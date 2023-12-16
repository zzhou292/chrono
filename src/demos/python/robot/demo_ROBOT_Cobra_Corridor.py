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
# Authors: Sriram Ashokkumar
# =============================================================================
#
# Demo to show Cobra Rover operated on Rigid Terrain
#
# =============================================================================

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot_chrono
import pychrono.sensor as sens

try:
   from pychrono import irrlicht as chronoirr
except:
   print('Could not import ChronoIrrlicht')

# Chreate Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

ground_mat = chrono.ChMaterialSurfaceNSC()


room_mmesh = chrono.ChTriangleMeshConnected()
room_mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/textures/hallway.obj"), False, True)
room_mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

room_trimesh_shape = chrono.ChTriangleMeshShape()
room_trimesh_shape.SetMesh(room_mmesh)
room_trimesh_shape.SetName("Hallway Mesh")
room_trimesh_shape.SetMutable(False)

room_mesh_body = chrono.ChBody()
room_mesh_body.SetPos(chrono.ChVectorD(-2, -2, -1))
room_mesh_body.AddVisualShape(room_trimesh_shape)
room_mesh_body.SetBodyFixed(True)
room_mesh_body.GetCollisionModel().ClearModel()

room_mesh_body.GetCollisionModel().AddTriangleMesh(ground_mat, room_mmesh, False, False)
room_mesh_body.GetCollisionModel().BuildModel()
room_mesh_body.SetCollide(True)

# vehicle.GetSystem().Add(room_mesh_body)
system.Add(room_mesh_body)


# # Create ground body
# ground_mat = chrono.ChMaterialSurfaceNSC()
# ground = chrono.ChBodyEasyBox(30, 30, 1, 1000, True, True, ground_mat)
# ground.SetPos(chrono.ChVectorD(2, 5, -1.415))
# ground.SetBodyFixed(True)
# ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
# system.Add(ground)

# Create Cobra rover
driver = robot_chrono.CobraSpeedDriver(1.0, 3.0)
rover = robot_chrono.Cobra(system, robot_chrono.CobraWheelType_SimpleWheel)
rover.SetDriver(driver)
rover.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, -0.2, -0.3), chrono.ChQuaternionD(1, 0, 0, 0)))

# Create the sensor 
sens_manager = sens.ChSensorManager(system)

cam_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0.18, 0, 0.35),
                              chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))


lidar_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0.0, 0, 0.4),
                              chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))

cam = sens.ChCameraSensor(rover.GetChassis().GetBody(), 30, cam_offset_pose, 1280,  720, 1.408,2)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
sens_manager.AddSensor(cam)

lidar = sens.ChLidarSensor(rover.GetChassis().GetBody(), 5., lidar_offset_pose, 90, 300,
                            2*chrono.CH_C_PI, chrono.CH_C_PI / 12, -chrono.CH_C_PI / 6, 100., 0)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))
sens_manager.AddSensor(lidar)

light_pos_1 = chrono.ChVectorD(0.0,0.0,1.89)
intensity = 1.0
sens_manager.scene.AddPointLight(chrono.ChVectorF(light_pos_1.x,light_pos_1.y,light_pos_1.z), chrono.ChColor(intensity, intensity, intensity), 500.0)

light_pos_2 = chrono.ChVectorD(12.0,0.0,1.89)
intensity = 1.0
sens_manager.scene.AddPointLight(chrono.ChVectorF(light_pos_2.x,light_pos_2.y,light_pos_2.z), chrono.ChColor(intensity, intensity, intensity), 500.0)


#lp_contact_material = chrono.ChMaterialSurfaceNSC()
#lp_mat = chrono.ChVisualMaterial()
#lp_mat.SetAmbientColor(chrono.ChColor(1., 0., 0.))
#lp_mat.SetDiffuseColor(chrono.ChColor(1., 0., 0.))

#light_body = chrono.ChBodyEasySphere(
#            0.2, 1000, True, False, lp_contact_material)

#light_body.SetPos(light_pos_2)
#light_body.SetBodyFixed(True)
#light_body.GetVisualShape(0).SetMaterial(0, lp_mat)
#system.Add(light_body)


# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Cobra rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2.5, 1.5), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)
####vis.EnableShadows()

time_step = 1e-3

driver.SetMotorSpeed(0)

# Simulation loop
time = 0

step = 0
while (vis.Run()) :
    time = time + time_step
    steering = 0
    driver.SetMotorSpeed(1.0)
    # if time > 7:
    # 	if abs(rover.GetTurnAngle()) < 1e-8:
    # 		steering = 0
    # 	else:
    # 		steering = -0.4
    # elif time > 1:
    # 	steering = 0.4
    # driver.SetSteering(steering)
    
    rover.Update()
    sens_manager.Update()

    if step%20==0 :
      vis.BeginScene()
      vis.Render()
      vis.EndScene()
    
    step =  step + 1

    system.DoStepDynamics(time_step)