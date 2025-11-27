# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
from cables import Model1, Model2, Model3
import pychrono.sensor as sens
import numpy as np

# Select solver type (SPARSE_QR, SPARSE_LU, or MINRES).
#ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR
solver = chrono.ChSolverSparseQR()

print("Copyright (c) 2017 projectchrono.org\nChrono version: ")

# Create a Chrono physical system
sys = chrono.ChSystemSMC()


# Create a mesh, that is a container for groups of elements and
# their referenced nodes.
mesh = fea.ChMesh()

# Create one of the available models (defined in FEAcables.h)
##model = Model1(sys, mesh)
##model = Model2(sys, mesh)
model = Model3(sys, mesh)

# Remember to add the mesh to the system!
sys.Add(mesh)

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChVisualShapeTriangleMesh).

colormap_type = chrono.ChColormap.Type_JET
colormap_range = chrono.ChVector2d(-0.01, 0.01)

vis_beam_A = chrono.ChVisualShapeFEA()
vis_beam_A.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_A.SetColormapRange(colormap_range)
vis_beam_A.SetSmoothFaces(True)
vis_beam_A.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_A)

vis_beam_B = chrono.ChVisualShapeFEA()
vis_beam_B.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS) # NODE_CSYS
vis_beam_B.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_B.SetSymbolsThickness(0.006)
vis_beam_B.SetSymbolsScale(0.01)
vis_beam_B.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_B)

mat0 = chrono.ChContactMaterialSMC();
mat0.SetFriction(0.1);
mat0.SetYoungModulus(2e7);
mrigidBody_floor_0 = chrono.ChBodyEasyBox(5, 5, 0.25, 
        400,         
        True,        
        True,        
        mat0);       
sys.Add(mrigidBody_floor_0);
mrigidBody_floor_0.SetPos(chrono.ChVector3d(0, 0, -1.0));
mrigidBody_floor_0.SetFixed(True);


# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()
vis.AddGuiColorbar('Mz (Nm)', colormap_range, colormap_type, False, chrono.ChVector2i(10, 100))

# Set solver and solver settings

if solver.GetType()==chrono.ChSolver.Type_SPARSE_QR:
	print("Using SparseQR solver")
	sys.SetSolver(solver)
	solver.UseSparsityPatternLearner(True)
	solver.LockSparsityPattern(True)
	solver.SetVerbose(False)

elif solver.GetType()== chrono.ChSolver.Type_MINRES :
	print( "Using MINRES solver" )
	sys.SetSolver(solver)
	solver.SetMaxIterations(200)
	solver.SetTolerance(1e-10)
	solver.EnableDiagonalPreconditioner(True)
	solver.EnableWarmStart(True)  # IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
	solver.SetVerbose(False)

else:
	print("Solver type not supported." )
    

# Set integrator
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)



### Add sensors
# Add camera sensor --------------------------------------------------------------------

lens_model = sens.PINHOLE
update_rate = 25
image_width = 256
image_height = 256
fov = 1.408
lag = 0
exposure_time = 0

manager = sens.ChSensorManager(sys)

intensity = 1.0
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1,0,0), chrono.ChVector3f(0,-1,0))


rotation_quat = chrono.QuatFromAngleAxis(np.pi*1/2, chrono.ChVector3d(1, 0, 0))
rotation_quat = rotation_quat * chrono.QuatFromAngleAxis(-np.pi*1/2, chrono.ChVector3d(0, 1, 0))
offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0.0, 0.0, 0.9), rotation_quat)

cam = sens.ChCameraSensor(
    mrigidBody_floor_0,              # body camera is attached to
    update_rate,            # update rate in Hz
    offset_pose,            # offset pose
    image_width,            # image width
    image_height,           # image height
    fov                    # camera's horizontal field of view
)
cam.SetName("Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
cam.PushFilter(sens.ChFilterVisualize(
    image_width, image_height, "Arm Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(cam) # Turned off


# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    manager.Update()
	##model.PrintBodyPositions()

