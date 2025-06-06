// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Base class for a steerable (rotary arm) DeDion solid axle suspension.
//
// This axle subsystem works with the ChRotaryArm steering subsystem.
//
// DeDion axles are lightweight solid axles, they don't carry a differential
// gearbox. There are tow guiding joints
//  - longitudinal: spherical
//  - lateral: point-to-plane (simplest version),
//             in real life often a Watt linkage is used instead
// Example cars with steerable DeDion axle
//  - Mowag Duro
//  - Mowag Eagle
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// suspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChToeBarDeDionAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChToeBarDeDionAxle::m_pointNames[] = {
    "SHOCK_A    ", "SHOCK_C    ", "KNUCKLE_L  ", "KNUCKLE_U  ", "KNUCKLE_DRL", "SPRING_A   ",
    "SPRING_C   ", "TIEROD_C   ", "TIEROD_K   ", "SPINDLE    ", "KNUCKLE_CM ", "AXLE_C     ",
    "WATT_CNT_LE", "WATT_CNT_RI", "WATT_LE_CH ", "WATT_RI_CH ", "STABI_CON  "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChToeBarDeDionAxle::ChToeBarDeDionAxle(const std::string& name) : ChSuspension(name) {}

ChToeBarDeDionAxle::~ChToeBarDeDionAxle() {
    if (!m_initialized)
        return;

    auto sys = m_axleTube->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_axleTube);
    sys->Remove(m_tierod);
    sys->Remove(m_draglink);
    sys->Remove(m_sphericalKnuckleTierod);
    sys->Remove(m_sphericalDraglinkPitman);
    sys->Remove(m_sphericalDraglinkKnuckle);
    sys->Remove(m_universalTierod);

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_knuckle[i]);
        sys->Remove(m_revoluteKingpin[i]);
        sys->Remove(m_shock[i]);
        sys->Remove(m_spring[i]);
    }
}

// -----------------------------------------------------------------------------
void ChToeBarDeDionAxle::Construct(std::shared_ptr<ChChassis> chassis,
                                   std::shared_ptr<ChSubchassis> subchassis,
                                   std::shared_ptr<ChSteering> steering,
                                   const ChVector3d& location,
                                   double left_ang_vel,
                                   double right_ang_vel) {
    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform the location of the axle body COM to absolute frame.
    ChVector3d axleCOM_local = getAxleTubeCOM();
    ChVector3d axleCOM = suspension_to_abs.TransformPointLocalToParent(axleCOM_local);

    // Calculate end points on the axle body, expressed in the absolute frame
    // (for visualization)
    ChVector3d midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
    ChVector3d outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    m_axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    ChVector3d conn_local(getLocation(STABI_CON));
    m_stabiConnectorL = suspension_to_abs.TransformPointLocalToParent(conn_local);
    conn_local.y() = -conn_local.y();
    m_stabiConnectorR = suspension_to_abs.TransformPointLocalToParent(conn_local);

    m_wattLower = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_LE));
    m_wattUpper = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_RI));
    m_wattOuterL = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_LE_CH));
    m_wattOuterR = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_RI_CH));

    // longitudinal guide, modeled as a pushbar
    m_axleChassis = suspension_to_abs.TransformPointLocalToParent(getLocation(AXLE_C));
    m_axleCenter = (m_axleOuterR + m_axleOuterL) / 2.0;

    // Create and initialize the axle body.
    m_axleTube = chrono_types::make_shared<ChBody>();
    m_axleTube->SetName(m_name + "_axleTube");
    m_axleTube->SetTag(m_obj_tag);
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTube);

    // Fix the axle body to the chassis
    m_axleTubeGuideLong = chrono_types::make_shared<ChLinkLockSpherical>();
    m_axleTubeGuideLong->SetName(m_name + "_sphereAxleTube");
    m_axleTubeGuideLong->SetTag(m_obj_tag);
    ChVector3d spPos = suspension_to_abs.TransformPointLocalToParent(getLocation(AXLE_C));
    m_axleTubeGuideLong->Initialize(m_axleTube, chassis->GetBody(), ChFrame<>(spPos, QUNIT));
    chassis->GetSystem()->AddLink(m_axleTubeGuideLong);

    // Watt lateral guiding mechanism
    ChVector3d cntrPos =
        suspension_to_abs.TransformPointLocalToParent((getLocation(WATT_CNT_LE) + getLocation(WATT_CNT_RI)) / 2.0);
    m_wattCenterLinkBody = chrono_types::make_shared<ChBody>();
    m_wattCenterLinkBody->SetName(m_name + "_wattCenterBody");
    m_wattCenterLinkBody->SetTag(m_obj_tag);
    m_wattCenterLinkBody->SetPos(cntrPos);
    m_wattCenterLinkBody->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_wattCenterLinkBody->SetMass(getWattCenterMass());
    m_wattCenterLinkBody->SetInertiaXX(getWattCenterInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_wattCenterLinkBody);

    ChVector3d lftPos =
        suspension_to_abs.TransformPointLocalToParent((getLocation(WATT_LE_CH) + getLocation(WATT_CNT_LE)) / 2.0);
    m_wattLeftLinkBody = chrono_types::make_shared<ChBody>();
    m_wattLeftLinkBody->SetName(m_name + "_wattLeftBody");
    m_wattLeftLinkBody->SetTag(m_obj_tag);
    m_wattLeftLinkBody->SetPos(lftPos);
    m_wattLeftLinkBody->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_wattLeftLinkBody->SetMass(getWattSideMass());
    m_wattLeftLinkBody->SetInertiaXX(getWattSideInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_wattLeftLinkBody);

    ChVector3d rghtPos =
        suspension_to_abs.TransformPointLocalToParent((getLocation(WATT_RI_CH) + getLocation(WATT_CNT_RI)) / 2.0);
    m_wattRightLinkBody = chrono_types::make_shared<ChBody>();
    m_wattRightLinkBody->SetName(m_name + "_wattRightBody");
    m_wattRightLinkBody->SetTag(m_obj_tag);
    m_wattRightLinkBody->SetPos(rghtPos);
    m_wattRightLinkBody->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_wattRightLinkBody->SetMass(getWattSideMass());
    m_wattRightLinkBody->SetInertiaXX(getWattSideInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_wattRightLinkBody);

    // link the Watt center link to the axle tube
    m_wattCenterRev = chrono_types::make_shared<ChLinkLockRevolute>();
    m_wattCenterRev->SetName(m_name + "_wattCenterPivot");
    m_wattCenterRev->SetTag(m_obj_tag);
    m_wattCenterRev->Initialize(m_wattCenterLinkBody, m_axleTube, ChFrame<>(cntrPos, QuatFromAngleY(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_wattCenterRev);

    // link the Watt left link to the center link
    m_wattLeftToCenterSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattLeftToCenterSph->SetName(m_name + "_wattLeft2CenterSph");
    m_wattLeftToCenterSph->SetTag(m_obj_tag);
    m_wattLeftToCenterSph->Initialize(
        m_wattLeftLinkBody, m_wattCenterLinkBody,
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_LE)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattLeftToCenterSph);

    // link the Watt left link to the axle tube
    m_wattLeftToAxleTubeSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattLeftToAxleTubeSph->SetName(m_name + "_wattLeft2ChassisSph");
    m_wattLeftToAxleTubeSph->SetTag(m_obj_tag);
    m_wattLeftToAxleTubeSph->Initialize(
        m_wattLeftLinkBody, chassis->GetBody(),
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_LE_CH)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattLeftToAxleTubeSph);

    // link the Watt right link to the center link
    m_wattRightToCenterSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattRightToCenterSph->SetName(m_name + "_wattRight2CenterSph");
    m_wattRightToCenterSph->SetTag(m_obj_tag);
    m_wattRightToCenterSph->Initialize(
        m_wattRightLinkBody, m_wattCenterLinkBody,
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_RI)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattRightToCenterSph);

    // link the Watt right link to the axle tube
    m_wattRightToAxleTubeSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattRightToAxleTubeSph->SetName(m_name + "_wattRight2ChassisSph");
    m_wattRightToAxleTubeSph->SetTag(m_obj_tag);
    m_wattRightToAxleTubeSph->Initialize(
        m_wattRightLinkBody, chassis->GetBody(),
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_RI_CH)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattRightToAxleTubeSph);

    // Calculate end points on the tierod body, expressed in the absolute frame
    // (for visualization)
    ChVector3d tierodOuter_local(getLocation(TIEROD_K));
    m_tierodOuterL = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);
    tierodOuter_local.y() = -tierodOuter_local.y();
    m_tierodOuterR = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);

    // Create and initialize the tierod body.
    m_tierod = chrono_types::make_shared<ChBody>();
    m_tierod->SetName(m_name + "_tierodBody");
    m_tierod->SetTag(m_obj_tag);
    m_tierod->SetPos((m_tierodOuterL + m_tierodOuterR) / 2);
    m_tierod->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_tierod->SetMass(getTierodMass());
    m_tierod->SetInertiaXX(getTierodInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_tierod);

    // Transform all hardpoints to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis->GetBody(), m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), m_pointsR, right_ang_vel);

    // Create connection to steering mechanism
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();

    if (isLeftKnuckleActuated()) {
        // Create and initialize the draglink body (one side only).
        // Determine the rotation matrix of the draglink based on the plane of the
        // hard points (z-axis along the length of the draglink)
        v = Vcross(m_pointsL[KNUCKLE_DRL], m_pointsL[DRAGLINK_C]);
        v.Normalize();
        w = m_pointsL[DRAGLINK_C] - m_pointsL[KNUCKLE_DRL];
        w.Normalize();
        u = Vcross(v, w);
        rot.SetFromDirectionAxes(u, v, w);

        m_draglink = chrono_types::make_shared<ChBody>();
        m_draglink->SetName(m_name + "_draglink");
        m_draglink->SetTag(m_obj_tag);
        m_draglink->SetPos((m_pointsL[DRAGLINK_C] + m_pointsL[KNUCKLE_DRL]) / 2);
        m_draglink->SetRot(rot.GetQuaternion());
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and
        // draglink.
        m_sphericalDraglinkPitman = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglinkPitman->SetName(m_name + "_sphericalDraglinkPitman" + "_L");
        m_sphericalDraglinkPitman->SetTag(m_obj_tag);
        m_sphericalDraglinkPitman->Initialize(m_draglink, tierod_body, ChFrame<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglinkPitman);

        // Create and initialize the universal joint between draglink and knuckle
        m_sphericalDraglinkKnuckle = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglinkKnuckle->SetName(m_name + "_sphericalDraglinkKnuckle" + "_L");
        m_sphericalDraglinkKnuckle->SetTag(m_obj_tag);
        m_sphericalDraglinkKnuckle->Initialize(m_draglink, m_knuckle[LEFT], ChFrame<>(m_pointsL[KNUCKLE_DRL], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglinkKnuckle);
    } else {
        // Create and initialize the draglink body (one side only).
        // Determine the rotation matrix of the draglink based on the plane of the
        // hard points (z-axis along the length of the draglink)
        v = Vcross(m_pointsR[KNUCKLE_DRL], m_pointsL[DRAGLINK_C]);
        v.Normalize();
        w = m_pointsL[DRAGLINK_C] - m_pointsR[KNUCKLE_DRL];
        w.Normalize();
        u = Vcross(v, w);
        rot.SetFromDirectionAxes(u, v, w);

        m_draglink = chrono_types::make_shared<ChBody>();
        m_draglink->SetName(m_name + "_draglink");
        m_draglink->SetTag(m_obj_tag);
        m_draglink->SetPos((m_pointsL[DRAGLINK_C] + m_pointsR[KNUCKLE_DRL]) / 2);
        m_draglink->SetRot(rot.GetQuaternion());
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and
        // draglink.
        m_sphericalDraglinkPitman = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglinkPitman->SetName(m_name + "_sphericalDraglinkPitman" + "_L");
        m_sphericalDraglinkPitman->SetTag(m_obj_tag);
        m_sphericalDraglinkPitman->Initialize(m_draglink, tierod_body, ChFrame<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglinkPitman);

        // Create and initialize the universal joint between draglink and knuckle
        m_sphericalDraglinkKnuckle = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglinkKnuckle->SetName(m_name + "_sphericalDraglinkKnuckle" + "_R");
        m_sphericalDraglinkKnuckle->SetTag(m_obj_tag);
        m_sphericalDraglinkKnuckle->Initialize(m_draglink, m_knuckle[RIGHT], ChFrame<>(m_pointsR[KNUCKLE_DRL], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglinkKnuckle);
    }
}

const ChVector3d ChToeBarDeDionAxle::GetConnectorLocation(VehicleSide side) {
    if (side == RIGHT) {
        return m_stabiConnectorR;
    } else {
        return m_stabiConnectorL;
    }
}

void ChToeBarDeDionAxle::InitializeSide(VehicleSide side,
                                        std::shared_ptr<ChBodyAuxRef> chassis,
                                        const std::vector<ChVector3d>& points,
                                        double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrameRefToAbs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * QuatFromAngleZ(sign * getToeAngle()) * QuatFromAngleX(sign * getCamberAngle());

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = chrono_types::make_shared<ChBody>();
    m_knuckle[side]->SetName(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetTag(m_obj_tag);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(spindleRot);
    m_knuckle[side]->SetMass(getKnuckleMass());
    m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
    chassis->GetSystem()->AddBody(m_knuckle[side]);

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = chrono_types::make_shared<ChBody>();
    m_spindle[side]->SetName(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize the joint between knuckle and tierod (one side has
    // universal, the other has spherical).
    if (side == LEFT) {
        m_sphericalKnuckleTierod = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalKnuckleTierod->SetName(m_name + "_sphericalTierod" + suffix);
        m_sphericalKnuckleTierod->SetTag(m_obj_tag);
        m_sphericalKnuckleTierod->Initialize(m_tierod, m_knuckle[side], ChFrame<>(points[TIEROD_K], QUNIT));
        chassis->GetSystem()->AddLink(m_sphericalKnuckleTierod);
    } else {
        m_universalTierod = chrono_types::make_shared<ChLinkUniversal>();
        m_universalTierod->SetName(m_name + "_universalTierod" + suffix);
        m_universalTierod->SetTag(m_obj_tag);
        m_universalTierod->Initialize(m_tierod, m_knuckle[side],
                                      ChFrame<>(points[TIEROD_K], chassisRot * QuatFromAngleX(CH_PI_2)));
        chassis->GetSystem()->AddLink(m_universalTierod);
    }

    // Create and initialize the revolute joint between axle and knuckle.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction.
    w = points[KNUCKLE_L] - points[KNUCKLE_U];
    w.Normalize();
    u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
    u.Normalize();
    v = Vcross(w, u);
    rot.SetFromDirectionAxes(u, v, w);

    m_revoluteKingpin[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetName(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->Initialize(m_axleTube, m_knuckle[side],
                                        ChFrame<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.GetQuaternion()));
    m_revoluteKingpin[side]->SetTag(m_obj_tag);
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the revolute joint between upright and spindle.
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->SetTag(m_obj_tag);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side],
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->SetTag(m_obj_tag);
    m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetName(m_name + "_spring" + suffix);
    m_spring[side]->SetTag(m_obj_tag);
    m_spring[side]->Initialize(chassis, m_axleTube, false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle.
    // Note that the spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetName(m_name + "_axle" + suffix);
    m_axle[side]->SetTag(m_obj_tag);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPosDt(-ang_vel);
    chassis->GetSystem()->AddShaft(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftBodyRotation>();
    m_axle_to_spindle[side]->SetName(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector3d(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

void ChToeBarDeDionAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + getTierodMass() + getDraglinkMass() + 2 * (getSpindleMass() + getKnuckleMass());
}

void ChToeBarDeDionAxle::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_axleTube->GetFrameCOMToAbs(), getAxleTubeMass(), ChMatrix33<>(getAxleTubeInertia()));
    composite.AddComponent(m_tierod->GetFrameCOMToAbs(), getTierodMass(), ChMatrix33<>(getTierodInertia()));
    composite.AddComponent(m_draglink->GetFrameCOMToAbs(), getDraglinkMass(), ChMatrix33<>(getDraglinkInertia()));
    composite.AddComponent(m_knuckle[LEFT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckle[RIGHT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChToeBarDeDionAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChToeBarDeDionAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarDeDionAxle::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarDeDionAxle::LogConstraintViolations(VehicleSide side) {
    // TODO: Update this to reflect new suspension joints
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteKingpin[side]->GetConstraintViolation();
        std::cout << "Kingpin revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    {
        ChVectorDynamic<> C = m_sphericalKnuckleTierod->GetConstraintViolation();
        std::cout << "Tierod spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalDraglinkPitman->GetConstraintViolation();
        std::cout << "Draglink spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }

    {
        ChVectorDynamic<> C = m_universalTierod->GetConstraintViolation();
        std::cout << "Tierod universal          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalDraglinkKnuckle->GetConstraintViolation();
        std::cout << "Draglink universal          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarDeDionAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_axleTube, m_axleCenter, m_axleChassis, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_axleTube, m_stabiConnectorL, m_stabiConnectorR, getAxleTubeRadius() / 2,
                         ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_wattCenterLinkBody, m_wattLower, m_wattUpper, getWattLinkRadius(),
                         ChColor(0.5f, 0.7f, 0.8f));
    AddVisualizationLink(m_wattLeftLinkBody, m_wattLower, m_wattOuterL, getWattLinkRadius(), ChColor(0.8f, 0.5f, 0.5f));
    AddVisualizationLink(m_wattRightLinkBody, m_wattUpper, m_wattOuterR, getWattLinkRadius(),
                         ChColor(0.5f, 0.8f, 0.5f));

    AddVisualizationLink(m_tierod, m_tierodOuterL, m_tierodOuterR, getTierodRadius(), ChColor(0.7f, 0.7f, 0.7f));

    if (isLeftKnuckleActuated()) {
        AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_C], m_pointsL[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    } else {
        AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_C], m_pointsR[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    }

    AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius());
    AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChToeBarDeDionAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);
    ChPart::RemoveVisualizationAssets(m_tierod);
    ChPart::RemoveVisualizationAssets(m_draglink);
    ChPart::RemoveVisualizationAssets(m_wattCenterLinkBody);
    ChPart::RemoveVisualizationAssets(m_wattLeftLinkBody);
    ChPart::RemoveVisualizationAssets(m_wattRightLinkBody);

    ChPart::RemoveVisualizationAssets(m_knuckle[LEFT]);
    ChPart::RemoveVisualizationAssets(m_knuckle[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarDeDionAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                              const ChVector3d pt_1,
                                              const ChVector3d pt_2,
                                              double radius,
                                              const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector3d p_2 = body->TransformPointParentToLocal(pt_2);

    auto cyl = utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
    cyl->SetColor(color);
}

void ChToeBarDeDionAxle::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                                 const ChVector3d pt_U,
                                                 const ChVector3d pt_L,
                                                 const ChVector3d pt_T,
                                                 double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector3d p_U = knuckle->TransformPointParentToLocal(pt_U);
    ChVector3d p_L = knuckle->TransformPointParentToLocal(pt_L);
    ChVector3d p_T = knuckle->TransformPointParentToLocal(pt_T);

    if (p_L.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_L, VNULL, radius);
    }

    if (p_U.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_U, VNULL, radius);
    }

    if (p_T.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_T, VNULL, radius);
    }
}

// -----------------------------------------------------------------------------

void ChToeBarDeDionAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    bodies.push_back(m_tierod);
    bodies.push_back(m_draglink);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalKnuckleTierod);
    joints.push_back(m_sphericalDraglinkPitman);
    joints.push_back(m_sphericalDraglinkKnuckle);
    joints.push_back(m_universalTierod);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChToeBarDeDionAxle::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    bodies.push_back(m_tierod);
    bodies.push_back(m_draglink);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalKnuckleTierod);
    joints.push_back(m_sphericalDraglinkPitman);
    joints.push_back(m_sphericalDraglinkKnuckle);
    joints.push_back(m_universalTierod);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
