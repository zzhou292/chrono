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
// Authors: Json Zhou
// =============================================================================
//
// Simple Chrono demo for the fully parameterized 2-node ANCF beam element 3243.
//
// Cantilever beam with a concentrated vertical tip load:
//   - Fz = 5000 N applied in global +Z at the free-end tip node
//   - held constant for steps 0..99
//   - released (set to 0) at step 100, with no external force for steps 100..199
//
// =============================================================================


// ./bin/jz_FEA_3243_check --nthreads 16 --res0 --msglvl 2

#include "chrono/ChConfig.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsNodeXYZ.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChOpenMP.h"

#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionSequence.h"

#if defined _WIN32 || defined _WIN64
    #include <windows.h>
#endif

using namespace chrono;
using namespace chrono::fea;

namespace {

constexpr double kE    = 7e8;
constexpr double kNu   = 0.33;
constexpr double kRho0 = 2700;
constexpr double kK1 = 1.0;
constexpr double kK2 = 1.0;
constexpr double kAlphaDamp = 0.0;

// Default per-element beam length (NOT total beam length).
constexpr double kDefaultL = 0.2;
constexpr double kDefaultW = 0.1;
constexpr double kDefaultH = 0.1;

struct DemoOptions {
    bool use_continuous_integration = true;
    int msglvl = 1;  // 0: quiet, 1: summary, 2: per-step

    int num_elements = 8;

    // Beam geometry:
    // - 'element_length' is the per-element beam length (dx).
    // - Total beam length is: L_total = element_length * num_elements.
    // - 'width_y' and 'height_z' are the cross-section dimensions applied to *each* element.
    double element_length = kDefaultL;  // m (per-element length, dx)
    double width_y = kDefaultW;   // m (cross-section width, local Y)
    double height_z = kDefaultH;  // m (cross-section height, local Z)

    double tip_force = 5000.0;  // N (Fz, applied along global +Z at the tip node)

    double step_size = 1e-3;  // s

    // Thread settings (0 means "auto", uses ChOMP::GetMaxThreads()).
    int nthreads = 0;

    bool write_csv = false;
    std::string csv_filename = "jz_FEA_3243_check_tip_z.csv";
};

static void PrintUsage(const char* exe) {
    std::cout << "Usage: " << exe << " [options]\n"
              << "Options:\n"
              << "  --contint           Use continuous integration (default)\n"
              << "  --preint            Use pre-integration method\n"
              << "  --elements N        Number of beam elements (default 8)\n"
              << "  --res {0,2,4,8,16,32}  Set number of elements (also: --res0, --res2, ...)\n"
              << "  --force F           Tip force magnitude in N (default 5000)\n"
              << "  --dt DT             Step size in s (default 1e-3)\n"
              << "  --nthreads N        Set threads (Chrono/collision/Eigen) (0=auto)\n"
              << "  --csv               Write per-step tip z to a CSV file\n"
              << "  --msglvl N          0 quiet, 1 summary, 2 per-step (default 1)\n"
              << "  --help              Show this message\n";
}

enum class ParseResult { Ok, ExitSuccess, ExitFailure };

static int NumElementsForRes(int res) {
    switch (res) {
        case 0:
            return 1000;
        case 2:
            return 10000;
        case 4:
            return 50000;
        case 8:
            return 100000;
        case 16:
            return 200000;
        case 32:
            return 500000;
    }
    return -1;
}

static ParseResult ParseArgs(int argc, char* argv[], DemoOptions& opt) {
    auto need_value = [&](int& i, const char* name) -> const char* {
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << name << "\n";
            return nullptr;
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];

        if (a == "--help" || a == "-h") {
            PrintUsage(argv[0]);
            return ParseResult::ExitSuccess;
        }
        if (a == "--contint") {
            opt.use_continuous_integration = true;
            continue;
        }
        if (a == "--preint") {
            opt.use_continuous_integration = false;
            continue;
        }
        if (a == "--elements") {
            const char* v = need_value(i, "--elements");
            if (!v)
                return ParseResult::ExitFailure;
            opt.num_elements = std::stoi(v);
            continue;
        }
        if (a == "--res") {
            const char* v = need_value(i, "--res");
            if (!v)
                return ParseResult::ExitFailure;
            const int res = std::stoi(v);
            const int elems = NumElementsForRes(res);
            if (elems < 0) {
                std::cerr << "Invalid --res " << res << " (allowed: 0,2,4,8,16,32)\n";
                return ParseResult::ExitFailure;
            }
            opt.num_elements = elems;
            continue;
        }
        if (a == "--res0" || a == "--res2" || a == "--res4" || a == "--res8" || a == "--res16" || a == "--res32") {
            const int res = std::stoi(a.substr(std::string("--res").size()));
            const int elems = NumElementsForRes(res);
            if (elems < 0) {
                std::cerr << "Invalid resolution flag: " << a << "\n";
                return ParseResult::ExitFailure;
            }
            opt.num_elements = elems;
            continue;
        }
        if (a == "--force") {
            const char* v = need_value(i, "--force");
            if (!v)
                return ParseResult::ExitFailure;
            opt.tip_force = std::stod(v);
            continue;
        }
        if (a == "--csv") {
            opt.write_csv = true;
            continue;
        }
        if (a == "--nthreads") {
            const char* v = need_value(i, "--nthreads");
            if (!v)
                return ParseResult::ExitFailure;
            opt.nthreads = std::stoi(v);
            continue;
        }
        if (a == "--dt") {
            const char* v = need_value(i, "--dt");
            if (!v)
                return ParseResult::ExitFailure;
            opt.step_size = std::stod(v);
            continue;
        }
        if (a == "--msglvl") {
            const char* v = need_value(i, "--msglvl");
            if (!v)
                return ParseResult::ExitFailure;
            opt.msglvl = std::stoi(v);
            continue;
        }

        std::cerr << "Unknown option: " << a << "\n";
        PrintUsage(argv[0]);
        return ParseResult::ExitFailure;
    }

    if (opt.num_elements < 1) {
        std::cerr << "--elements must be >= 1\n";
        return ParseResult::ExitFailure;
    }
    if (opt.step_size <= 0) {
        std::cerr << "--dt must be > 0\n";
        return ParseResult::ExitFailure;
    }
    if (opt.nthreads < 0) {
        std::cerr << "--nthreads must be >= 0 (0 means auto)\n";
        return ParseResult::ExitFailure;
    }
    return ParseResult::Ok;
}

static std::shared_ptr<ChElementBeamANCF_3243> BuildCantileverBeam(ChSystemSMC& system,
                                                                   const DemoOptions& opt,
                                                                   std::shared_ptr<ChNodeFEAxyzDDD>& tip_node_out) {
    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(kRho0, kE, kNu, kK1, kK2);

    auto mesh = chrono_types::make_shared<ChMesh>();
    system.Add(mesh);
    mesh->SetAutomaticGravity(false);

    const double dx = opt.element_length;
    // NOTE: dx is the per-element length. Increasing num_elements increases the total beam length.

    // Setup beam cross section gradients to initially align with global axes.
    const ChVector3d dir1(1, 0, 0);
    const ChVector3d dir2(0, 1, 0);
    const ChVector3d dir3(0, 0, 1);

    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(0, 0, 0.0), dir1, dir2, dir3);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    auto elementlast = chrono_types::make_shared<ChElementBeamANCF_3243>();
    for (int i = 1; i <= opt.num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(dx * i, 0, 0), dir1, dir2, dir3);
        mesh->AddNode(nodeB);

        auto element = chrono_types::make_shared<ChElementBeamANCF_3243>();
        element->SetNodes(nodeA, nodeB);
        element->SetDimensions(dx, opt.width_y, opt.height_z);
        element->SetMaterial(material);
        element->SetAlphaDamp(kAlphaDamp);

        if (!opt.use_continuous_integration) {
            element->SetIntFrcCalcMethod(ChElementBeamANCF_3243::IntFrcMethod::PreInt);
        }

        mesh->AddElement(element);
        nodeA = nodeB;
        elementlast = element;
    }

    tip_node_out = nodeA;
    system.Update(chrono::UpdateFlags::UPDATE_ALL);
    return elementlast;
}

static void AddTipNodeLoad(ChSystemSMC& system,
                           const DemoOptions& opt,
                           const std::shared_ptr<ChNodeFEAxyzDDD>& tip_node,
                           int total_steps,
                           int release_step) {
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);

    auto tip_load = chrono_types::make_shared<ChLoadNodeXYZForceAbs>(tip_node, ChVector3d(0, 0, opt.tip_force));

    // Step-based schedule (with fixed dt): hold until 'release_step', then release.
    const double t_release = release_step * opt.step_size;
    const double t_total = total_steps * opt.step_size;
    const double dur_on = (t_release > 0) ? t_release : 0.0;
    const double dur_off = (t_total > t_release) ? (t_total - t_release) : 0.0;

    auto modulation = chrono_types::make_shared<ChFunctionSequence>();
    modulation->InsertFunct(chrono_types::make_shared<ChFunctionConst>(1.0), dur_on);
    modulation->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.0), dur_off);
    modulation->SetClamped(true);
    modulation->Setup();

    tip_load->SetModulationFunction(modulation);
    loadcontainer->Add(tip_load);
}

}  // namespace

int main(int argc, char* argv[]) {
    DemoOptions opt;
    const auto parse_result = ParseArgs(argc, argv, opt);
    if (parse_result == ParseResult::ExitSuccess)
        return 0;
    if (parse_result == ParseResult::ExitFailure)
        return 1;

#if defined _WIN32 || defined _WIN64
    // Windows console with ANSI colors handling
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hOut, &dwMode);
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, dwMode);
#endif

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    const int auto_threads = std::max(1, ChOMP::GetMaxThreads());
    const int nthreads = (opt.nthreads > 0) ? opt.nthreads : auto_threads;
    system.SetNumThreads(nthreads, nthreads, nthreads);
    ChOMP::SetNumThreads(nthreads);

    if (opt.msglvl >= 1) {
#ifdef _OPENMP
        std::cout << "OpenMP enabled (max_threads=" << ChOMP::GetMaxThreads() << ")\n";
#else
        std::cout << "OpenMP disabled at compile time\n";
#endif
        std::cout << "Thread settings: nthreads=" << nthreads << " (chrono/collision/eigen)\n";
    }

    // Linear solver settings (iterative solver => tolerance applies).
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);
	    solver->SetMaxIterations(1000);
	    const double solver_tol = (opt.num_elements == NumElementsForRes(32)) ? 1e-3 : 1e-4;
	    solver->SetTolerance(solver_tol);

	    solver->EnableDiagonalPreconditioner(true);
	    solver->SetVerbose(false);

    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    const int release_step = 100;
    const int total_steps = 200;

    std::shared_ptr<ChNodeFEAxyzDDD> tip_node;
    auto tip_element = BuildCantileverBeam(system, opt, tip_node);
    AddTipNodeLoad(system, opt, tip_node, total_steps, release_step);

    if (opt.msglvl >= 1) {
        const double total_length = opt.element_length * opt.num_elements;
        std::cout << "ANCF 3243 cantilever demo\n";
        std::cout << "  integration : " << (opt.use_continuous_integration ? "ContInt" : "PreInt") << "\n";
        std::cout << "  solver      : " << system.GetSolver()->GetTypeAsString();
        if (auto it = system.GetSolver()->AsIterative()) {
            std::cout << " (tol=" << it->GetTolerance() << ", maxiters=" << it->GetMaxIterations() << ")";
        }
        std::cout << "\n";
        std::cout << "  elements    : " << opt.num_elements << "\n";
        std::cout << "  L_total     : " << total_length << " m\n";
        std::cout << "  dx          : " << opt.element_length << " m (per-element)\n";
        std::cout << "  W,H         : " << opt.width_y << ", " << opt.height_z << " m\n";
        std::cout << "  tip_force   : " << opt.tip_force << " N (Fz, applied along +Z)\n";
        std::cout << "  schedule    : Fz=const for steps 0.." << (release_step - 1) << ", then Fz=0 for steps "
                  << release_step << ".." << (total_steps - 1) << "\n";
        std::cout << "  dt          : " << opt.step_size << " s\n";
        std::cout << "  steps       : " << total_steps << " (t_end=" << (total_steps * opt.step_size) << " s)\n\n";
    }

    if (opt.msglvl >= 2) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "time, tip_z\n";
    }

    std::vector<std::pair<double, double>> csv_rows;
    if (opt.write_csv) {
        csv_rows.reserve(total_steps);
    }

    const auto wall_start = std::chrono::steady_clock::now();
    for (int step = 0; step < total_steps; step++) {
        system.DoStepDynamics(opt.step_size);

        if (opt.write_csv || opt.msglvl >= 2) {

            const double tip_x = tip_node->GetPos().x();
            const double tip_y = tip_node->GetPos().y();
            const double tip_z = tip_node->GetPos().z();

            const double time = system.GetChTime();

            if (opt.write_csv) {
                csv_rows.emplace_back(time, tip_z);
            }
            if (opt.msglvl >= 2) {
                std::cout << time << ", " << tip_x << ", " << tip_y << ", " << tip_z << "\n";

            }
        }
    }
    const auto wall_end = std::chrono::steady_clock::now();

    const double wall_s = std::chrono::duration<double>(wall_end - wall_start).count();
    const double sim_s = total_steps * opt.step_size;
    const double rtf = (sim_s > 0) ? (wall_s / sim_s) : 0.0;

    if (opt.msglvl >= 1) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Wall time (loop)     : " << wall_s << " s\n";
        std::cout << "Simulation time      : " << sim_s << " s\n";
        std::cout << "RTF (wall/sim)       : " << rtf << "\n\n";
    }

    if (opt.write_csv) {
        std::ofstream of(opt.csv_filename);
        if (!of) {
            std::cerr << "ERROR: cannot open CSV file '" << opt.csv_filename << "' for writing\n";
        } else {
            of << "time,tip_z\n";
            of << std::fixed << std::setprecision(12);
            for (const auto& [time, tip_z] : csv_rows) {
                of << time << "," << tip_z << "\n";
            }
            if (opt.msglvl >= 1) {
                std::cout << "Wrote CSV: " << opt.csv_filename << " (" << csv_rows.size() << " rows)\n\n";
            }
        }
    }

    ChVector3d tip_pos;
    ChQuaternion<> tip_rot;
    tip_element->EvaluateSectionFrame(1.0, tip_pos, tip_rot);
    const ChVector3d tip_angles = tip_rot.GetCardanAnglesXYZ();

    if (opt.msglvl >= 1) {
        std::cout << "Final tip position   : " << tip_pos << " m\n";
        std::cout << "Final tip angles XYZ : " << (tip_angles * CH_RAD_TO_DEG) << " deg\n";
    }

    return 0;
}
