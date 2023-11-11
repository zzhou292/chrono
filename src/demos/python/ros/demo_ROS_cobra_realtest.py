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
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

contact_method = chrono.ChMaterialSurfaceNSC()

room_mmesh = chrono.ChTriangleMeshConnected()
room_mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("robot/environment/room_2/test_REAL.obj"), False, True)
room_mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

room_trimesh_shape = chrono.ChTriangleMeshShape()
room_trimesh_shape.SetMesh(room_mmesh)
room_trimesh_shape.SetName("Hallway Mesh")
room_trimesh_shape.SetMutable(False)

room_mesh_body = chrono.ChBody()
room_mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
room_mesh_body.AddVisualShape(room_trimesh_shape)
room_mesh_body.SetBodyFixed(True)
room_mesh_body.GetCollisionModel().ClearModel()

room_mesh_body.GetCollisionModel().AddTriangleMesh(contact_method, room_mmesh, False, False)
room_mesh_body.GetCollisionModel().BuildModel()
room_mesh_body.SetCollide(True)

# vehicle.GetSystem().Add(room_mesh_body)
system.Add(room_mesh_body)

# Create Cobra rover
driver = robot_chrono.CobraSpeedDriver(1.0, 3.0)
rover = robot_chrono.Cobra(system, robot_chrono.CobraWheelType_SimpleWheel)
rover.SetDriver(driver)
rover.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.4), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))

quat = chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 0, 1))
print(quat)

# Create the sensor 
sens_manager = sens.ChSensorManager(system)

offset_pose1 = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.2),
                              chrono.Q_from_AngAxis(chrono.CH_C_PI/18, chrono.ChVectorD(0, 1, 0)))

cam = sens.ChCameraSensor(rover.GetChassis().GetBody(), 30, offset_pose1, 1280,  720, 1.408)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
sens_manager.AddSensor(cam)

offset_pose2 = chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0.6),
                              chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))

lidar = sens.ChLidarSensor(rover.GetChassis().GetBody(), 5., offset_pose2, 512, 64,
                            2*chrono.CH_C_PI, chrono.CH_C_PI / 4, -chrono.CH_C_PI / 4, 100., 0)
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
    lidar, "/input_cloud"))
ros_manager.RegisterHandler(chros.ChROSCobraSpeedDriverHandler(25, driver, "/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, rover.GetChassis().GetBody(), "~/output/cobra/state"))


tf_rate = 50
tf_topic_name = "/tf"
ros_manager.RegisterHandler(chros.ChROSTFHandler(
    tf_rate, lidar, rover.GetChassis().GetBody(), tf_topic_name))
ros_manager.Initialize()

####vis.EnableShadows()

time_step = 5e-4

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
    #driver.SetMotorSpeed(2)
    
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