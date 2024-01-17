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
// Simple engine model for the HMMWV vehicle based on torque-speed engine maps
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimpleMap.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

const double rpm2rads = CH_C_PI / 30;

HMMWV_EngineSimpleMap::HMMWV_EngineSimpleMap(const std::string& name) : ChEngineSimpleMap(name) {}

double HMMWV_EngineSimpleMap::GetMaxEngineSpeed() {
    return 2700 * rpm2rads;
}

void HMMWV_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    double rpm_to_radsec = CH_C_2PI / 60.;
    map0.AddPoint(0, 0.000);

    mapF.AddPoint(0 * rpm_to_radsec, 0);  // to start engine
    mapF.AddPoint(500 * rpm_to_radsec, 400);
    mapF.AddPoint(5000 * rpm_to_radsec, 400);
    mapF.AddPoint(9000 * rpm_to_radsec, 0);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
