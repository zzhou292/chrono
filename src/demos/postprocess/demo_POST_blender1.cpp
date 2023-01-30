// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about using the assets system to create shapes that can be shown in
// the 3D postprocessing, by using Blender.
//
// - The chrono_import.py add-on must be installed in Blender add-ons.
// - run this exe to generate files in ./DEMO_OUTPUT/BLENDER_ directory
// - Use the Blender menu File/Import/Chrono Import to load the assets.py file saved by this exe.
//
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChObjFileShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_postprocess/ChBlender.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace postprocess;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    // Create an exporter to Blender
    ChBlender blender_exporter = ChBlender(&sys);

    // Set the path where it will save all files, a directory will be created if not
    // existing
    blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_1");

    // Optional: change the default naming of the generated files:
    // blender_exporter.SetOutputDataFilebase("state");
    // blender_exporter.SetPictureFilebase("picture");

    // --Optional: add further commands
    //   Remember to use \ at the end of each line for a multiple-line string. Pay attention to tabs in Python.
    blender_exporter.SetCustomBlenderCommandsScript(
"\
# Test \
");

 
    /* Start example */
    /// [Example 1]

    // Create a ChBody, and attach some 'assets'
    // that define 3D shapes. These shapes can be shown
    // by Irrlicht or POV postprocessing, etc...
    // Note: these assets are independent from collision shapes!

    // Create a rigid body as usual, and add it
    // to the physical system:
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);

    // Define a collision shape
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    floor->GetCollisionModel()->ClearModel();
    floor->GetCollisionModel()->AddBox(floor_mat, 10, 0.5, 10, ChVector<>(0, -1, 0));
    floor->GetCollisionModel()->BuildModel();
    floor->SetCollide(true);

    // Add body to system
    sys.Add(floor);

    // ==Asset== attach a 'box' shape.
    auto boxfloor = chrono_types::make_shared<ChBoxShape>();
    boxfloor->GetBoxGeometry().Size = ChVector<>(10, 0.5, 10);
    boxfloor->SetColor(ChColor(0.3f, 0.3f, 0.6f));
    floor->AddVisualShape(boxfloor, ChFrame<>(ChVector<>(0, -1, 0)));

    /// [Example 1]
    /* End example */

    /* Start example */
    /// [Example 2]

    // Textures, colors, asset levels with transformations.

    // Create the rigid body as usual (this won't move, it is only for visualization tests)
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(true);
    sys.Add(body);

    // ==Asset== Attach a 'box' shape
    auto mbox = chrono_types::make_shared<ChBoxShape>();
    mbox->GetBoxGeometry().Size = ChVector<>(0.2, 0.5, 0.1);
    body->AddVisualShape(mbox, ChFrame<>(ChVector<>(1, 0, 0)));

    // ==Asset== Attach a 'cylinder' shape
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(3, 1, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(4, 2, 0);
    cyl->GetCylinderGeometry().rad = 0.2;
    body->AddVisualShape(cyl);
    // ...here is an example on how to change the color:
    cyl->SetColor(ChColor(1.f, 0.8f, 0.f));

    // ==Asset== Attach a 'sphere' shape
    auto sphere = chrono_types::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.5;
    body->AddVisualShape(sphere, ChFrame<>(ChVector<>(-1, 0, 0)));

    // ...btw here is an example on how to setup a material:
    auto visual_material = chrono_types::make_shared<ChVisualMaterial>();
    visual_material->SetMetallic(0.5);
    visual_material->SetRoughness(0.1);
    sphere->AddMaterial(visual_material);

    // ==Asset== Attach a 'Wavefront mesh' asset, referencing a .obj file:
    auto objmesh = chrono_types::make_shared<ChObjFileShape>();
    objmesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));
    objmesh->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    body->AddVisualShape(objmesh, ChFrame<>(ChVector<>(0, 0, 2)));

    // ==Asset== Attach an array of boxes, each rotated to make a spiral
    for (int j = 0; j < 20; j++) {
        auto smallbox = chrono_types::make_shared<ChBoxShape>();
        smallbox->GetBoxGeometry().Size = ChVector<>(0.1, 0.1, 0.01);
        smallbox->SetColor(ChColor(j * 0.05f, 1 - j * 0.05f, 0.0f));
        ChMatrix33<> rot(Q_from_AngY(j * 21 * CH_C_DEG_TO_RAD));
        ChVector<> pos = rot * ChVector<>(0.4, 0, 0) + ChVector<>(0, j * 0.02, 0);
        body->AddVisualShape(smallbox, ChFrame<>(pos, rot));
    }

    // ==Asset== Attach a 'triangle mesh, defined via vertexes & triangle faces. Here, 4 vertexes, 2 faces 
    auto trimesh = chrono_types::make_shared<ChTriangleMeshShape>();
      // ...four vertices
    trimesh->GetMesh()->getCoordsVertices() = std::vector<chrono::ChVector<>>{
        {2,1,0},
        {3,1,0},
        {3,2,0},
        {2,2,0}
    };
      // ...two triangle faces, whose indexes point to the vertexes above. Counterclockwise.
    trimesh->GetMesh()->getIndicesVertexes() = std::vector<chrono::ChVector<int>>{
        {0,1,2},
        {2,3,0},
    };
      // ... one normal, pointing toward Y (NOTE: normals would be unnecessary in Blender - here just for completeness)
    trimesh->GetMesh()->getCoordsNormals() = std::vector<chrono::ChVector<>>{
        {0,0,1},
    };
      // ... same normal for all vertexes of both triangles (NOTE: normals would be unnecessary in Blender, etc.)
    trimesh->GetMesh()->getIndicesNormals() = std::vector<chrono::ChVector<int>>{
        {0,0,0},
        {0,0,0}
    };
      // ... per-vertex colors, RGB:
    trimesh->GetMesh()->getCoordsColors() = std::vector<chrono::ChColor>{
        {0.9,0.8,0.1},
        {0.8,0.2,0.3},
        {0.2,0.1,0.9},
        {0.2,0.6,0.6}
    };

     // NOTE: optionally, you can add a scalar or vector property, per vertex or per face, that can
     // be rendered via falsecolor in Blender:
    geometry::ChPropertyScalar my_scalars;
    my_scalars.name = "temperature";
    my_scalars.data = std::vector<double>{ 0, 10, 100, 120 };
    trimesh->GetMesh()->AddPropertyPerVertex(my_scalars);

    geometry::ChPropertyVector my_vectors;
    my_vectors.name = "velocity";
    my_vectors.data = std::vector<ChVector<>>{ {0,1,2}, {3,0,0}, {0,0,2}, {0,0,0} };
    trimesh->GetMesh()->AddPropertyPerVertex(my_vectors);

    body->AddVisualShape(trimesh, ChFrame<>(ChVector<>(-4, 0, 0)));


    // ==Asset== Attach a video camera.
    // Note that a camera can also be attached to a moving object.
    auto camera = chrono_types::make_shared<ChCamera>();
    camera->SetAngle(50);
    camera->SetPosition(ChVector<>(-3, 4, -5));
    camera->SetAimPoint(ChVector<>(0, 1, 0));
    body->AddCamera(camera);

    // Otherwise a simple (not attached to object) static camera can be optionally added via
    // blender_exporter.SetCamera(ChVector<>(3, 4, -5), ChVector<>(0, 0.5, 0), 50); , see later.
    

    /// [Example 2]
    /* End example */

    /* Start example */
    /// [Example 3]

    // Create a ChParticleClones cluster, and attach 'assets'
    // that define a single "sample" 3D shape. This will be shown
    // N times in Irrlicht.

    // Create the ChParticleClones, populate it with some random particles,
    // and add it to physical system:
    auto particles = chrono_types::make_shared<ChParticleCloud>();

    // Note: coll. shape, if needed, must be specified before creating particles
    auto particle_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    particles->GetCollisionModel()->ClearModel();
    particles->GetCollisionModel()->AddSphere(particle_mat, 0.05);
    particles->GetCollisionModel()->BuildModel();
    particles->SetCollide(true);

    // Create the random particles
    for (int np = 0; np < 100; ++np)
        particles->AddParticle(ChCoordsys<>(ChVector<>(ChRandom() - 2, 1, ChRandom() - 0.5)));

    // Do not forget to add the particle cluster to the system:
    sys.Add(particles);

    //  ==Asset== Attach a 'sphere' shape asset.. it will be used as a sample
    // shape to display all particles when rendering in 3D!
    auto sphereparticle = chrono_types::make_shared<ChSphereShape>();
    sphereparticle->GetSphereGeometry().rad = 0.05;
    particles->AddVisualShape(sphereparticle);

    /// [Example 3]
    /* End example */

    // Export all existing visual shapes to POV-Ray
    blender_exporter.AddAll();

    // (Optional: tell selectively which physical items you
    // want to render in the folllowing way...)
    //	blender_exporter.RemoveAll();
    //	blender_exporter.Add(floor);
    //	blender_exporter.Add(body);
    //	blender_exporter.Add(particles);

    // Set a default camera (this is an easier but less  powerful method than 
    // attaching a ChCamera to a ChBody via my_body->AddCamera(..) , see above.
    blender_exporter.SetCamera(ChVector<>(3, 4, -5), ChVector<>(0, 0.5, 0), 50); // pos, aim, angle

    // Turn on 3d XYZ axis for each body reference frame, and set xyz symbol size
    blender_exporter.SetShowItemsFrames(true, 0.3);

    // Turn on exporting contacts
    blender_exporter.SetShowContacts(true, 
                        0.02,   // scaling factor for lenght of the vector arrow
                        0.02,   // thickness of the vector arrow
                        true, 0, 10); // true=use colormap, and min and max values for colormap  

    //
    // RUN THE SIMULATION AND SAVE THE POVray FILES AT EACH FRAME
    //

    // 1) Create the two .pov and .ini files for POV-Ray (this must be done
    //    only once at the beginning of the simulation).

    blender_exporter.ExportScript();

    while (sys.GetChTime() < 1.5) {
        sys.DoStepDynamics(0.01);

        GetLog() << "time= " << sys.GetChTime() << "\n";

        // 2) Create the incremental nnnnn.dat and nnnnn.pov files in output/ dir, that will be load
        //    by the Blender chrono_import add-on.
        blender_exporter.ExportData();
    }

    // That's all! If all worked ok, this python script should
    // have created a  "xxxx.asset.py"  file that you can
    // load in Blender.

    return 0;
}