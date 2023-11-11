#
# BSD 3-Clause License
#
# Copyright (c) 2022 University of Wisconsin - Madison
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#

# General Imports
import numpy as np
import os

# Chrono Imports
import pychrono as chrono
import pychrono.vehicle as veh
# import pychrono.irrlicht as irr
import pychrono.sensor as sens


try:
   from pychrono import irrlicht as chronoirr
except:
   print('Could not import ChronoIrrlicht')

#// =============================================================================

#// =============================================================================
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

contact_method = chrono.ChContactMethod_NSC
chassis_collision_type = veh.CollisionType_NONE

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_PRIMITIVES
tire_vis_type = veh.VisualizationType_PRIMITIVES

tire_model = veh.TireModelType_TMEASY

# Initial vehicle location
init_loc_x = -2.5
init_loc_y = 0.5
init_angle_z = 1.57

initLoc = chrono.ChVectorD(init_loc_x, init_loc_y, 0.5)
initRot = chrono.Q_from_AngZ(init_angle_z)

step_size = 1e-3
tire_step_size = step_size

# Create the RCCar vehicle, set parameters, and initialize
vehicle = veh.ARTcar(system)
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetTireType(tire_model)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)
vehicle.SetTireVisualizationType(tire_vis_type)

driver = veh.ChDriver(vehicle.GetVehicle())

# ground_mat = chrono.ChMaterialSurfaceNSC()
# ground = chrono.ChBodyEasyBox(30, 30, 1, 1000, True, True, ground_mat)
# ground.SetPos(chrono.ChVectorD(2, 5, -1.415))
# ground.SetBodyFixed(True)
# ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
# system.Add(ground)

# Create the terrain
# terrain = veh.RigidTerrain(vehicle.GetSystem())

# # patch_mat = chrono.ChMaterialSurfaceNSC()
# # patch_mat.SetFriction(0.9)
# # patch_mat.SetRestitution(0.01) #-----check if mu, cr, and y is set similar to minfo

# # # not right need to introduce point cloud -- not sure how to access this using the package_share_directory
# # patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, chrono.GetChronoDataFile("autonomy-toolkit/me3038/rm3038_pt_cloud.obj"))  #chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
# # patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
# # patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
# terrain.Initialize()

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


# veh.ChTerrain(system, veh.GetDataFile("terrain/RigidPlane.json"))
# terrain.Initialize()

# === create sensors ===
# Create the sensor 
sens_manager = sens.ChSensorManager(system)

offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8, 0, 2),
                            chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))

cam = sens.ChCameraSensor(vehicle.GetChassis().GetBody(), 30, offset_pose, 1280,  720, 1.408)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
sens_manager.AddSensor(cam)


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


time_step = 1e-3

time = 0

step = 0
while (vis.Run()) :
    driver.SetThrottle(10)
    time = time + time_step


    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)

    driver.Advance(time_step)
    vehicle.Advance(time_step)
    terrain.Advance(time_step)
    sens_manager.Update()


    if step%20==0 :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        step =  step + 1

        system.DoStepDynamics(time_step)
