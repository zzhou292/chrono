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
// Authors: Radu Serban
// =============================================================================
//
// HMMWV engine model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineShafts.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// Static variables
const double HMMWV_EngineShafts::m_motorblock_inertia = 10.5;
const double HMMWV_EngineShafts::m_motorshaft_inertia = 1.1;

HMMWV_EngineShafts::HMMWV_EngineShafts(const std::string& name) : ChEngineShafts(name, ChVector<>(1, 0, 0)) {}

void HMMWV_EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(0 * rpm_to_radsec, 0);  // to start engine
    map->AddPoint(500 * rpm_to_radsec, 400);
    map->AddPoint(5000 * rpm_to_radsec, 400);
    map->AddPoint(9000 * rpm_to_radsec, 0);
}

void HMMWV_EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    double rpm_to_radsec = CH_C_2PI / 60.;

    map->AddPoint(0 * rpm_to_radsec, 0);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
