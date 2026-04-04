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
// Authors: OpenAI Codex
// =============================================================================
//
// Rigid-body double pendulum intended to match the FEAT10
// engineering_joint/test_feat10_double_pendulum_revolute scenario as closely as
// possible with ideal rigid links and ideal revolute joints.
//
// Key frame convention:
// - Each link is a ChBodyAuxRef.
// - The body reference frame (REF) is the beam-local frame with origin at the
//   proximal hinge and +Z along the beam axis.
// - The centroidal frame (COG) is offset by (0, 0, 0.25) from REF.
// - The supplied inertia tensor is specified at the COM and expressed in the
//   beam-local frame, which is valid here because REF and COG frames are
//   parallel.
//
// Joint convention:
// - Chrono revolute joints rotate about the joint frame Z axis.
// - The physical hinge axis must be global +Y, so each joint frame is rotated
//   by -90 deg about X, mapping joint +Z onto world +Y.
//
// The FE TetGen mesh is not loaded here. For this rigid-body comparison, the
// dynamics use the supplied mass and inertia directly, while visualization uses
// an equivalent 0.04 x 0.04 x 0.5 box from the mesh bounds.
//
// Output:
// - CSV file "demo_MBS_feat10_double_pendulum.csv" in the run directory.
// - Columns include the requested absolute/relative angles, lower tip position,
//   and joint reactions in the joint-local frame.
//
// =============================================================================

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkRevolute.h"
#include "chrono/physics/ChSystemNSC.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

using namespace chrono;

namespace {

const double kStepSize = 5e-4;
const int kNumSteps = 5000;

const double kLinkMass = 0.96;
const double kLinkLength = 0.5;
const double kLinkWidth = 0.04;
const double kLinkThickness = 0.04;
const ChVector3d kLinkInertiaDiag(0.020128, 0.020128, 0.000256);
const ChVector3d kLinkInertiaOffDiag(0, 0, 0);

const ChVector3d kBeamCOGLocal(0, 0, 0.25);
const ChVector3d kBeamDistalLocal(0, 0, 0.5);

const ChVector3d kTopHingeWorld(0, 0, 0.7);
const ChVector3d kInterBodyHingeWorld(0.28678822, 0, 0.29042398);
const ChVector3d kUpperCOMWorld(0.14339411, 0, 0.49521199);
const ChVector3d kLowerCOMWorld(0.18113365, 0, 0.06384703);
const ChVector3d kLowerTipWorld0(0.07547909, 0, -0.16272992);

const double kUpperAngleDeg = 145.0;
const double kLowerAngleDeg = 205.0;

double WrapAngleRadians(double angle) {
    while (angle <= -CH_PI) {
        angle += CH_2PI;
    }
    while (angle > CH_PI) {
        angle -= CH_2PI;
    }
    return angle;
}

double DirectionAngleXZ(const ChVector3d& direction) {
    return std::atan2(direction.x(), direction.z());
}

ChVector3d LinkDirectionWorld(const ChBodyAuxRef& body) {
    const ChVector3d proximal = body.GetFrameRefToAbs().TransformPointLocalToParent(VNULL);
    const ChVector3d distal = body.GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);
    return (distal - proximal).GetNormalized();
}

std::shared_ptr<ChBodyAuxRef> CreateLinkBody(const std::string& name,
                                             const ChVector3d& com_pos,
                                             const ChQuaterniond& ref_rot,
                                             const ChColor& color) {
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetName(name);
    body->SetFixed(false);
    body->EnableCollision(false);
    body->SetMass(kLinkMass);
    body->SetInertiaXX(kLinkInertiaDiag);
    body->SetInertiaXY(kLinkInertiaOffDiag);
    body->SetFrameCOMToRef(ChFrame<>(kBeamCOGLocal, QUNIT));
    body->SetPos(com_pos);
    body->SetRot(ref_rot);

    auto box = chrono_types::make_shared<ChVisualShapeBox>(kLinkWidth, kLinkThickness, kLinkLength);
    box->SetColor(color);
    body->AddVisualShape(box, ChFrame<>(kBeamCOGLocal, QUNIT));

    return body;
}

void WriteCsvHeader(std::ofstream& csv) {
    csv << "time"
        << ",upper_abs_angle_deg"
        << ",lower_abs_angle_deg"
        << ",relative_angle_deg"
        << ",lower_tip_x"
        << ",lower_tip_y"
        << ",lower_tip_z"
        << ",lower_tip_speed"
        << ",joint1_react_fx"
        << ",joint1_react_fy"
        << ",joint1_react_fz"
        << ",joint1_react_fmag"
        << ",joint1_react_tx"
        << ",joint1_react_ty"
        << ",joint1_react_tz"
        << ",joint2_react_fx"
        << ",joint2_react_fy"
        << ",joint2_react_fz"
        << ",joint2_react_fmag"
        << ",joint2_react_tx"
        << ",joint2_react_ty"
        << ",joint2_react_tz"
        << "\n";
}

void ReportInitialGeometry(const std::shared_ptr<ChBodyAuxRef>& upper, const std::shared_ptr<ChBodyAuxRef>& lower) {
    const ChVector3d upper_hinge = upper->GetFrameRefToAbs().TransformPointLocalToParent(VNULL);
    const ChVector3d upper_tip = upper->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);
    const ChVector3d lower_hinge = lower->GetFrameRefToAbs().TransformPointLocalToParent(VNULL);
    const ChVector3d lower_tip = lower->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);

    std::cout << std::scientific << std::setprecision(3);
    std::cout << "Setup check:\n";
    std::cout << "  |upper hinge - O| = " << (upper_hinge - kTopHingeWorld).Length() << "\n";
    std::cout << "  |upper tip - A|   = " << (upper_tip - kInterBodyHingeWorld).Length() << "\n";
    std::cout << "  |lower hinge - A| = " << (lower_hinge - kInterBodyHingeWorld).Length() << "\n";
    std::cout << "  |lower tip - B|   = " << (lower_tip - kLowerTipWorld0).Length() << "\n";
    std::cout << std::defaultfloat;
}

void LogState(std::ofstream& csv,
              ChSystemNSC& sys,
              const std::shared_ptr<ChBodyAuxRef>& upper,
              const std::shared_ptr<ChBodyAuxRef>& lower,
              const std::shared_ptr<ChLinkRevolute>& joint1,
              const std::shared_ptr<ChLinkRevolute>& joint2) {
    const ChVector3d upper_dir = LinkDirectionWorld(*upper);
    const ChVector3d lower_dir = LinkDirectionWorld(*lower);
    const double upper_angle = DirectionAngleXZ(upper_dir);
    const double lower_angle = DirectionAngleXZ(lower_dir);
    const double relative_angle = WrapAngleRadians(lower_angle - upper_angle);

    const ChVector3d lower_tip = lower->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);
    const ChVector3d lower_tip_vel = lower->GetFrameRefToAbs().PointSpeedLocalToParent(kBeamDistalLocal);

    const auto j1_reaction = joint1->GetReaction2();
    const auto j2_reaction = joint2->GetReaction2();
    const ChVector3d& j1_f = j1_reaction.force;
    const ChVector3d& j1_t = j1_reaction.torque;
    const ChVector3d& j2_f = j2_reaction.force;
    const ChVector3d& j2_t = j2_reaction.torque;
    const double j1_fmag = j1_f.Length();
    const double j2_fmag = j2_f.Length();

    csv << sys.GetChTime() << ","
        << upper_angle * CH_RAD_TO_DEG << ","
        << lower_angle * CH_RAD_TO_DEG << ","
        << relative_angle * CH_RAD_TO_DEG << ","
        << lower_tip.x() << ","
        << lower_tip.y() << ","
        << lower_tip.z() << ","
        << lower_tip_vel.Length() << ","
        << j1_f.x() << ","
        << j1_f.y() << ","
        << j1_f.z() << ","
        << j1_fmag << ","
        << j1_t.x() << ","
        << j1_t.y() << ","
        << j1_t.z() << ","
        << j2_f.x() << ","
        << j2_f.y() << ","
        << j2_f.z() << ","
        << j2_fmag << ","
        << j2_t.x() << ","
        << j2_t.y() << ","
        << j2_t.z() << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::cout << "Chrono version: " << CHRONO_VERSION << "\n";
    std::cout << "Rigid FEAT10-style double pendulum demo\n";

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys.SetSolverType(ChSolver::Type::PSOR);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetName("ground");
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sys.AddBody(ground);

    auto ground_box = chrono_types::make_shared<ChVisualShapeBox>(0.08, 0.08, 0.08);
    ground_box->SetColor(ChColor(0.35f, 0.35f, 0.35f));
    ground->AddVisualShape(ground_box, ChFrame<>(kTopHingeWorld, QUNIT));

    const ChQuaterniond upper_rot = QuatFromAngleY(kUpperAngleDeg * CH_DEG_TO_RAD);
    const ChQuaterniond lower_rot = QuatFromAngleY(kLowerAngleDeg * CH_DEG_TO_RAD);

    auto upper = CreateLinkBody("upper_link", kUpperCOMWorld, upper_rot, ChColor(0.75f, 0.15f, 0.15f));
    auto lower = CreateLinkBody("lower_link", kLowerCOMWorld, lower_rot, ChColor(0.15f, 0.15f, 0.75f));
    sys.Add(upper);
    sys.Add(lower);

    const ChQuaterniond joint_rot = QuatFromAngleX(-CH_PI_2);

    auto joint1 = chrono_types::make_shared<ChLinkRevolute>();
    joint1->Initialize(ground, upper, ChFrame<>(kTopHingeWorld, joint_rot));
    sys.AddLink(joint1);

    auto joint2 = chrono_types::make_shared<ChLinkRevolute>();
    joint2->Initialize(upper, lower, ChFrame<>(kInterBodyHingeWorld, joint_rot));
    sys.AddLink(joint2);

    ReportInitialGeometry(upper, lower);

    std::ofstream csv("demo_MBS_feat10_double_pendulum.csv");
    csv << std::setprecision(16);
    WriteCsvHeader(csv);

    LogState(csv, sys, upper, lower, joint1, joint2);

#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("FEAT10 Double Pendulum");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(1.6, -2.4, 1.1), kTopHingeWorld);
    vis->AddTypicalLights();

    ChRealtimeStepTimer realtime_timer;
    int step = 0;
    while (vis->Run() && step < kNumSteps) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(kStepSize);
        LogState(csv, sys, upper, lower, joint1, joint2);
        realtime_timer.Spin(kStepSize);
        ++step;
    }
#else
    for (int step = 0; step < kNumSteps; ++step) {
        sys.DoStepDynamics(kStepSize);
        LogState(csv, sys, upper, lower, joint1, joint2);
    }
#endif

    const ChVector3d upper_com = upper->GetPos();
    const ChVector3d lower_com = lower->GetPos();
    const ChVector3d lower_tip = lower->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);
    const double upper_angle = DirectionAngleXZ(LinkDirectionWorld(*upper)) * CH_RAD_TO_DEG;
    const double lower_angle = DirectionAngleXZ(LinkDirectionWorld(*lower)) * CH_RAD_TO_DEG;

    std::cout << std::fixed << std::setprecision(8);
    std::cout << "Initial/reference geometry is encoded through ChBodyAuxRef REF frames.\n";
    std::cout << "Final time: " << sys.GetChTime() << " s\n";
    std::cout << "Upper COM: (" << upper_com.x() << ", " << upper_com.y() << ", " << upper_com.z() << ")\n";
    std::cout << "Lower COM: (" << lower_com.x() << ", " << lower_com.y() << ", " << lower_com.z() << ")\n";
    std::cout << "Upper abs angle: " << upper_angle << " deg\n";
    std::cout << "Lower abs angle: " << lower_angle << " deg\n";
    std::cout << "Lower tip: (" << lower_tip.x() << ", " << lower_tip.y() << ", " << lower_tip.z() << ")\n";
    std::cout << "CSV output: demo_MBS_feat10_double_pendulum.csv\n";
    std::cout << "Joint reactions are written in each joint's local frame; the local +Z axis is the hinge axis (+Y in world).\n";

    return 0;
}
