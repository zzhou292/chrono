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
// Rigid-body 3D double pendulum with spherical joints, matching the supplied
// FEAT10-style geometry and mass properties.
//
// Frame convention used here:
// - Each beam is a ChBodyAuxRef.
// - The body REF frame is the beam geometric frame, with origin at the
//   proximal hinge and +Z along the beam centerline.
// - The body COM frame is offset from REF by (0, 0, 0.25).
//
// This choice lets the spherical-joint attachment points be expressed directly
// as REF-local coordinates VNULL (proximal hinge) and (0, 0, 0.5) (distal
// hinge), while still assigning inertia about the COM.
//
// =============================================================================

#include <algorithm>
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
#include "chrono/physics/ChLinkLock.h"
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

const ChVector3d kBeamCOMLocal(0, 0, 0.25);
const ChVector3d kBeamDistalLocal(0, 0, 0.5);
const ChVector3d kDownWorld(0, 0, -1);

const ChVector3d kTopHingeWorld(0, 0, 0.7);
const ChVector3d kInterBodyHingeWorld(0.28678822, 0.12656595, 0.31047006);
const ChVector3d kUpperCOMWorld(0.14339411, 0.06328298, 0.50523503);
const ChVector3d kLowerCOMWorld(0.18113365, 0.02370217, 0.10858852);
const ChVector3d kLowerTipWorld0(0.07547909, -0.07916161, -0.09329302);

const ChVector3d kUpperDir0(0.57357644, 0.25313190, -0.77905989);
const ChVector3d kLowerDir0(-0.42261826, -0.41145513, -0.80752615);

// Chrono quaternions use (w, x, y, z). These values were checked against the
// supplied direction vectors and produce the requested beam centerline axes.
const ChQuaterniond kUpperRot0(0.29700361, 0.04704075, 0.94197511, 0.14919420);
const ChQuaterniond kLowerRot0(-0.21045937, 0.05052682, 0.94932087, -0.22791178);

double ClampCosine(double value) {
    return std::clamp(value, -1.0, 1.0);
}

double SwingAngle(const ChVector3d& direction) {
    return std::acos(ClampCosine(direction.GetNormalized().Dot(kDownWorld)));
}

double Azimuth(const ChVector3d& direction) {
    return std::atan2(direction.y(), direction.x());
}

double InterLinkAngle(const ChVector3d& dir1, const ChVector3d& dir2) {
    return std::acos(ClampCosine(dir1.GetNormalized().Dot(dir2.GetNormalized())));
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
    body->SetFrameCOMToRef(ChFrame<>(kBeamCOMLocal, QUNIT));
    body->SetPos(com_pos);
    body->SetRot(ref_rot);

    auto box = chrono_types::make_shared<ChVisualShapeBox>(kLinkWidth, kLinkThickness, kLinkLength);
    box->SetColor(color);
    body->AddVisualShape(box, ChFrame<>(kBeamCOMLocal, QUNIT));

    return body;
}

void ReportInitialGeometry(const std::shared_ptr<ChBodyAuxRef>& upper, const std::shared_ptr<ChBodyAuxRef>& lower) {
    const ChVector3d upper_hinge = upper->GetFrameRefToAbs().TransformPointLocalToParent(VNULL);
    const ChVector3d upper_tip = upper->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);
    const ChVector3d lower_hinge = lower->GetFrameRefToAbs().TransformPointLocalToParent(VNULL);
    const ChVector3d lower_tip = lower->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);
    const ChVector3d upper_dir = LinkDirectionWorld(*upper);
    const ChVector3d lower_dir = LinkDirectionWorld(*lower);

    std::cout << std::scientific << std::setprecision(3);
    std::cout << "Setup check:\n";
    std::cout << "  |upper hinge - O| = " << (upper_hinge - kTopHingeWorld).Length() << "\n";
    std::cout << "  |upper tip - A|   = " << (upper_tip - kInterBodyHingeWorld).Length() << "\n";
    std::cout << "  |lower hinge - A| = " << (lower_hinge - kInterBodyHingeWorld).Length() << "\n";
    std::cout << "  |lower tip - B|   = " << (lower_tip - kLowerTipWorld0).Length() << "\n";
    std::cout << "  |upper dir - du|  = " << (upper_dir - kUpperDir0).Length() << "\n";
    std::cout << "  |lower dir - dl|  = " << (lower_dir - kLowerDir0).Length() << "\n";
    std::cout << std::defaultfloat;
}

void WriteCsvHeader(std::ofstream& csv) {
    csv << "time"
        << ",upper_swing_deg"
        << ",upper_azimuth_deg"
        << ",lower_swing_deg"
        << ",lower_azimuth_deg"
        << ",inter_link_angle_deg"
        << ",lower_tip_x"
        << ",lower_tip_y"
        << ",lower_tip_z"
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

void LogState(std::ofstream& csv,
              ChSystemNSC& sys,
              const std::shared_ptr<ChBodyAuxRef>& upper,
              const std::shared_ptr<ChBodyAuxRef>& lower,
              const std::shared_ptr<ChLinkLockSpherical>& joint1,
              const std::shared_ptr<ChLinkLockSpherical>& joint2) {
    const ChVector3d upper_dir = LinkDirectionWorld(*upper);
    const ChVector3d lower_dir = LinkDirectionWorld(*lower);

    const double upper_swing = SwingAngle(upper_dir);
    const double upper_azimuth = Azimuth(upper_dir);
    const double lower_swing = SwingAngle(lower_dir);
    const double lower_azimuth = Azimuth(lower_dir);
    const double inter_link = InterLinkAngle(upper_dir, lower_dir);

    const ChVector3d lower_tip = lower->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);

    const auto j1_reaction = joint1->GetReaction2();
    const auto j2_reaction = joint2->GetReaction2();
    const ChVector3d& j1_f = j1_reaction.force;
    const ChVector3d& j1_t = j1_reaction.torque;
    const ChVector3d& j2_f = j2_reaction.force;
    const ChVector3d& j2_t = j2_reaction.torque;

    csv << sys.GetChTime() << ","
        << upper_swing * CH_RAD_TO_DEG << ","
        << upper_azimuth * CH_RAD_TO_DEG << ","
        << lower_swing * CH_RAD_TO_DEG << ","
        << lower_azimuth * CH_RAD_TO_DEG << ","
        << inter_link * CH_RAD_TO_DEG << ","
        << lower_tip.x() << ","
        << lower_tip.y() << ","
        << lower_tip.z() << ","
        << j1_f.x() << ","
        << j1_f.y() << ","
        << j1_f.z() << ","
        << j1_f.Length() << ","
        << j1_t.x() << ","
        << j1_t.y() << ","
        << j1_t.z() << ","
        << j2_f.x() << ","
        << j2_f.y() << ","
        << j2_f.z() << ","
        << j2_f.Length() << ","
        << j2_t.x() << ","
        << j2_t.y() << ","
        << j2_t.z() << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::cout << "Chrono version: " << CHRONO_VERSION << "\n";
    std::cout << "Rigid FEAT10-style double pendulum with spherical joints\n";

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys.SetSolverType(ChSolver::Type::PSOR);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetName("ground");
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sys.AddBody(ground);

    auto support_marker = chrono_types::make_shared<ChVisualShapeBox>(0.08, 0.08, 0.08);
    support_marker->SetColor(ChColor(0.35f, 0.35f, 0.35f));
    ground->AddVisualShape(support_marker, ChFrame<>(kTopHingeWorld, QUNIT));

    auto upper = CreateLinkBody("upper_link", kUpperCOMWorld, kUpperRot0, ChColor(0.75f, 0.15f, 0.15f));
    auto lower = CreateLinkBody("lower_link", kLowerCOMWorld, kLowerRot0, ChColor(0.15f, 0.15f, 0.75f));
    sys.Add(upper);
    sys.Add(lower);

    auto joint1 = chrono_types::make_shared<ChLinkLockSpherical>();
    joint1->Initialize(ground, upper, ChFrame<>(kTopHingeWorld, QUNIT));
    sys.AddLink(joint1);

    auto joint2 = chrono_types::make_shared<ChLinkLockSpherical>();
    joint2->Initialize(upper, lower, ChFrame<>(kInterBodyHingeWorld, QUNIT));
    sys.AddLink(joint2);

    ReportInitialGeometry(upper, lower);

    std::ofstream csv("demo_MBS_feat10_double_pendulum_spherical.csv");
    csv << std::setprecision(16);
    WriteCsvHeader(csv);
    LogState(csv, sys, upper, lower, joint1, joint2);

#ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("FEAT10 Double Pendulum Spherical");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(1.8, -2.6, 1.2), kInterBodyHingeWorld);
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

    const ChVector3d upper_dir = LinkDirectionWorld(*upper);
    const ChVector3d lower_dir = LinkDirectionWorld(*lower);
    const ChVector3d lower_tip = lower->GetFrameRefToAbs().TransformPointLocalToParent(kBeamDistalLocal);

    std::cout << std::fixed << std::setprecision(8);
    std::cout << "Final time: " << sys.GetChTime() << " s\n";
    std::cout << "Upper swing: " << SwingAngle(upper_dir) * CH_RAD_TO_DEG << " deg\n";
    std::cout << "Upper azimuth: " << Azimuth(upper_dir) * CH_RAD_TO_DEG << " deg\n";
    std::cout << "Lower swing: " << SwingAngle(lower_dir) * CH_RAD_TO_DEG << " deg\n";
    std::cout << "Lower azimuth: " << Azimuth(lower_dir) * CH_RAD_TO_DEG << " deg\n";
    std::cout << "Inter-link angle: " << InterLinkAngle(upper_dir, lower_dir) * CH_RAD_TO_DEG << " deg\n";
    std::cout << "Lower tip: (" << lower_tip.x() << ", " << lower_tip.y() << ", " << lower_tip.z() << ")\n";
    std::cout << "CSV output: demo_MBS_feat10_double_pendulum_spherical.csv\n";

    return 0;
}
