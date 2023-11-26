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
import pychrono.ros as chros
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

contact_method = chrono.ChMaterialSurfaceNSC()


room_mmesh = chrono.ChTriangleMeshConnected()
room_mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/textures/hallway.obj"), False, True)
room_mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

room_trimesh_shape = chrono.ChVisualShapeTriangleMesh()
room_trimesh_shape.SetMesh(room_mmesh)
room_trimesh_shape.SetName("Hallway Mesh")
room_trimesh_shape.SetMutable(False)

room_mesh_body = chrono.ChBody()
room_mesh_body.SetPos(chrono.ChVectorD(-2, -2, -1))
room_mesh_body.AddVisualShape(room_trimesh_shape)
room_mesh_body.SetBodyFixed(True)

cshape = chrono.ChCollisionShapeTriangleMesh(contact_method, room_mmesh, True, True)
room_mesh_body.AddCollisionShape(cshape)
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

offset_pose = chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 1.2),
                              chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))

cam = sens.ChCameraSensor(rover.GetChassis().GetBody(), 30, offset_pose, 1280,  720, 1.408)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
sens_manager.AddSensor(cam)

lidar = sens.ChLidarSensor(rover.GetChassis().GetBody(), 5., offset_pose, 90, 300,
                            2*chrono.CH_C_PI, chrono.CH_C_PI / 12, -chrono.CH_C_PI / 6, 100., 0)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))
sens_manager.AddSensor(lidar)


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

# Create ROS manager
ros_manager = chros.ChROSManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

ros_manager.RegisterHandler(chros.ChROSCameraHandler(
        cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(
    lidar, "~/output/lidar/data/pointcloud"))
ros_manager.RegisterHandler(chros.ChROSTFHandler(50, lidar, rover.GetChassis().GetBody(), "/tf"))
ros_manager.RegisterHandler(chros.ChROSCobraSpeedDriverHandler(25, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, rover.GetChassis().GetBody(), "~/output/cobra/state"))
ros_manager.Initialize()

####vis.EnableShadows()

time_step = 1e-3

driver.SetMotorSpeed(0)

# Simulation loop
time = 0

step = 0
while (vis.Run()) :
    time = time + time_step
    steering = 0
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
    if not ros_manager.Update(time, time_step):
        break
    if step%20==0 :
      vis.BeginScene()
      vis.Render()
      vis.EndScene()
    
    step =  step + 1

    system.DoStepDynamics(time_step)