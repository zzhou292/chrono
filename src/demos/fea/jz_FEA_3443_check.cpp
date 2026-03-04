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
// Simple Chrono demo for the fully parameterized 4-node ANCF shell element 3443.
//
// Cantilever shell panel with a distributed vertical tip load:
//   - Total Fz = -500 N applied uniformly along the free-end tip edge (x = x_size)
//   - held constant for steps 0..99
//   - released (set to 0) at step 100, with no external force for steps 100..199
//
// Shell panel fixed dimensions: x_size=4.0 m, y_size=2.0 m, thickness=0.1 m
// Grid resolution controlled by --res {0,2,4,8,16,32}:
//   0  ->  10x10  elements  (100 elements total)
//   2  ->  20x20  elements  (400 elements total)
//   4  ->  50x50  elements  (2500 elements total)
//   8  -> 100x100 elements  (10000 elements total)
//   16 -> 150x150 elements  (22500 elements total)
//   32 -> 200x200 elements  (40000 elements total)
//
// This mirrors the scenario in:
//   Total-Lagrangian-FEA/lib_bin/beam_sag/test_ancf3443.cc
//
// =============================================================================

// ./bin/jz_FEA_3443_check --csv --dt 1e-3 --nthreads 8 --res0 --msglvl 2

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

#include "chrono/fea/ChElementShellANCF_3443.h"
#include "chrono/fea/ChMaterialShellANCF.h"
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

// Material constants (matching GPU library test_ancf3443)
constexpr double kE         = 7e8;
constexpr double kNu        = 0.33;
constexpr double kRho0      = 2700;
constexpr double kAlphaDamp = 0.0;

// Fixed shell total dimensions (same as GPU library: kShellXSize=4, kShellYSize=2, kDefaultH=0.1)
constexpr double kShellXSize = 4.0;
constexpr double kShellYSize = 2.0;
constexpr double kShellH     = 0.1;

// Default total tip force: matches GPU library default = -5000 * kDefaultH = -500 N
// Negative = downward (-Z direction)
constexpr double kDefaultTipForceZ = -5000.0 * kShellH;

struct DemoOptions {
    bool use_continuous_integration = true;
    int msglvl = 1;  // 0: quiet, 1: summary, 2: per-step

    // Resolution index: 0, 2, 4, 8, 16, 32
    // Maps to nx=ny: 10, 20, 50, 100, 150, 200
    int res = 0;

    // Shell total dimensions (fixed to match GPU library)
    double x_size    = kShellXSize;
    double y_size    = kShellYSize;
    double thickness = kShellH;

    // Total vertical force applied uniformly along the free tip edge (x = x_size)
    double tip_force_z = kDefaultTipForceZ;

    double step_size = 5e-4;  // s

    // Thread settings (0 means "auto", uses ChOMP::GetMaxThreads())
    int nthreads = 0;

    bool write_csv = false;
    std::string csv_filename = "jz_FEA_3443_check_tip_z.csv";
};

// Maps the 'res' flag to the grid dimension (nx = ny = return value).
static int GridResolutionFromRes(int res) {
    switch (res) {
        case 0:  return 10;
        case 2:  return 20;
        case 4:  return 50;
        case 8:  return 100;
        case 16: return 150;
        case 32: return 200;
    }
    return -1;
}

static void PrintUsage(const char* exe) {
    std::cout << "Usage: " << exe << " [options]\n"
              << "Options:\n"
              << "  --contint             Use continuous integration (default)\n"
              << "  --preint              Use pre-integration method\n"
              << "  --res {0,2,4,8,16,32}  Grid resolution (default 0)\n"
              << "                          0->10x10, 2->20x20, 4->50x50,\n"
              << "                          8->100x100, 16->150x150, 32->200x200\n"
              << "  --force F             Total tip force in N (default -500)\n"
              << "  --dt DT               Step size in s (default 1e-3)\n"
              << "  --nthreads N          Set threads (0=auto)\n"
              << "  --csv                 Write per-step average tip z to a CSV file\n"
              << "  --msglvl N            0 quiet, 1 summary, 2 per-step (default 1)\n"
              << "  --help                Show this message\n";
}

enum class ParseResult { Ok, ExitSuccess, ExitFailure };

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
        if (a == "--res") {
            const char* v = need_value(i, "--res");
            if (!v)
                return ParseResult::ExitFailure;
            const int res = std::stoi(v);
            if (GridResolutionFromRes(res) < 0) {
                std::cerr << "Invalid --res " << res << " (allowed: 0,2,4,8,16,32)\n";
                return ParseResult::ExitFailure;
            }
            opt.res = res;
            continue;
        }
        if (a == "--res0" || a == "--res2" || a == "--res4" || a == "--res8" || a == "--res16" ||
            a == "--res32") {
            const int res = std::stoi(a.substr(std::string("--res").size()));
            if (GridResolutionFromRes(res) < 0) {
                std::cerr << "Invalid resolution flag: " << a << "\n";
                return ParseResult::ExitFailure;
            }
            opt.res = res;
            continue;
        }
        if (a == "--force") {
            const char* v = need_value(i, "--force");
            if (!v)
                return ParseResult::ExitFailure;
            opt.tip_force_z = std::stod(v);
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

// Build a cantilever shell mesh in the XY plane.
//   - (nx+1)*(ny+1) nodes over [0, x_size] x [0, y_size] at z=0
//   - Nodes at x=0 are fixed (clamped edge)
//   - tip_nodes_out receives all nodes at x=x_size (free edge)
static void BuildCantileverShell(ChSystemSMC& system,
                                  const DemoOptions& opt,
                                  std::vector<std::shared_ptr<ChNodeFEAxyzDDD>>& tip_nodes_out) {
    const int grid_res = GridResolutionFromRes(opt.res);
    const int nx = grid_res;   // number of elements along X
    const int ny = grid_res;   // number of elements along Y
    const int N_x = nx + 1;   // nodes along X
    const int N_y = ny + 1;   // nodes along Y

    const double dx = opt.x_size / nx;
    const double dy = opt.y_size / ny;

    // Isotropic material (uses single-arg E,nu constructor)
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(kRho0, kE, kNu);

    auto mesh = chrono_types::make_shared<ChMesh>();
    system.Add(mesh);
    mesh->SetAutomaticGravity(false);

    // Gradient directions for a flat shell initially in the XY plane.
    // dir_x = dr/dx = (1,0,0), dir_y = dr/dy = (0,1,0), dir_z = dr/dz = (0,0,1)
    const ChVector3d dir_x(1, 0, 0);
    const ChVector3d dir_y(0, 1, 0);
    const ChVector3d dir_z(0, 0, 1);

    // Create nodes in row-major order: index = iy * N_x + ix
    std::vector<std::shared_ptr<ChNodeFEAxyzDDD>> nodes(N_x * N_y);
    for (int iy = 0; iy < N_y; iy++) {
        for (int ix = 0; ix < N_x; ix++) {
            const double x = ix * dx;
            const double y = iy * dy;
            auto node = chrono_types::make_shared<ChNodeFEAxyzDDD>(
                ChVector3d(x, y, 0.0), dir_x, dir_y, dir_z);
            node->SetMass(0);
            // Clamp the left edge (x = 0)
            if (ix == 0) {
                node->SetFixed(true);
            }
            mesh->AddNode(node);
            nodes[iy * N_x + ix] = node;
        }
    }

    // Collect all tip-edge nodes (at x = x_size, i.e., ix == nx)
    tip_nodes_out.clear();
    tip_nodes_out.reserve(N_y);
    for (int iy = 0; iy < N_y; iy++) {
        tip_nodes_out.push_back(nodes[iy * N_x + nx]);
    }

    // Create elements.
    // Node ordering (CCW, matching ChElementShellANCF_3443 convention):
    //   A = lower-left  (ix,   iy  )   -> u_min, v_min
    //   B = lower-right (ix+1, iy  )   -> u_max, v_min
    //   C = upper-right (ix+1, iy+1)   -> u_max, v_max
    //   D = upper-left  (ix,   iy+1)   -> u_min, v_max
    for (int iy = 0; iy < ny; iy++) {
        for (int ix = 0; ix < nx; ix++) {
            const int nA = iy       * N_x + ix;
            const int nB = iy       * N_x + ix + 1;
            const int nC = (iy + 1) * N_x + ix + 1;
            const int nD = (iy + 1) * N_x + ix;

            auto element = chrono_types::make_shared<ChElementShellANCF_3443>();
            element->SetNodes(nodes[nA], nodes[nB], nodes[nC], nodes[nD]);
            element->SetDimensions(dx, dy);
            element->AddLayer(opt.thickness, 0.0, mat);  // single layer, 0 deg fiber angle
            element->SetAlphaDamp(kAlphaDamp);

            if (!opt.use_continuous_integration) {
                element->SetIntFrcCalcMethod(ChElementShellANCF_3443::IntFrcMethod::PreInt);
            }

            mesh->AddElement(element);
        }
    }

    system.Update(chrono::UpdateFlags::UPDATE_ALL);
}

// Distribute the total tip_force_z uniformly across all tip-edge nodes.
// Uses a modulation function: force on for steps 0..release_step-1, off after.
static void AddTipEdgeLoad(ChSystemSMC& system,
                           const DemoOptions& opt,
                           const std::vector<std::shared_ptr<ChNodeFEAxyzDDD>>& tip_nodes,
                           int total_steps,
                           int release_step) {
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);

    const double fz_per_node =
        opt.tip_force_z / static_cast<double>(tip_nodes.size());

    const double t_release = release_step * opt.step_size;
    const double t_total   = total_steps  * opt.step_size;
    const double dur_on    = (t_release > 0) ? t_release : 0.0;
    const double dur_off   = (t_total > t_release) ? (t_total - t_release) : 0.0;

    // Shared modulation function across all tip loads
    auto modulation = chrono_types::make_shared<ChFunctionSequence>();
    modulation->InsertFunct(chrono_types::make_shared<ChFunctionConst>(1.0), dur_on);
    modulation->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.0), dur_off);
    modulation->SetClamped(true);
    modulation->Setup();

    for (const auto& node : tip_nodes) {
        auto tip_load = chrono_types::make_shared<ChLoadNodeXYZForceAbs>(
            node, ChVector3d(0, 0, fz_per_node));
        tip_load->SetModulationFunction(modulation);
        loadcontainer->Add(tip_load);
    }
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

    // Linear solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);
    solver->SetMaxIterations(1000);
    const double solver_tol = (opt.res >= 16) ? 1e-3 : 1e-4;
    solver->SetTolerance(solver_tol);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    const int release_step = 100;
    const int total_steps  = 200;

    const int grid_res       = GridResolutionFromRes(opt.res);
    const int nx             = grid_res;
    const int ny             = grid_res;
    const int total_elements = nx * ny;
    const int total_nodes    = (nx + 1) * (ny + 1);

    std::vector<std::shared_ptr<ChNodeFEAxyzDDD>> tip_nodes;
    BuildCantileverShell(system, opt, tip_nodes);
    AddTipEdgeLoad(system, opt, tip_nodes, total_steps, release_step);

    if (opt.msglvl >= 1) {
        const double dx = opt.x_size / nx;
        const double dy = opt.y_size / ny;
        std::cout << "ANCF 3443 cantilever shell demo\n";
        std::cout << "  integration  : " << (opt.use_continuous_integration ? "ContInt" : "PreInt") << "\n";
        std::cout << "  solver       : " << system.GetSolver()->GetTypeAsString();
        if (auto it = system.GetSolver()->AsIterative()) {
            std::cout << " (tol=" << it->GetTolerance() << ", maxiters=" << it->GetMaxIterations() << ")";
        }
        std::cout << "\n";
        std::cout << "  res          : " << opt.res << " (" << nx << "x" << ny << " elements)\n";
        std::cout << "  elements     : " << total_elements << "\n";
        std::cout << "  nodes        : " << total_nodes << "\n";
        std::cout << "  tip_nodes    : " << tip_nodes.size() << "\n";
        std::cout << "  x_size,y_size: " << opt.x_size << ", " << opt.y_size << " m\n";
        std::cout << "  dx, dy       : " << dx << ", " << dy << " m\n";
        std::cout << "  thickness    : " << opt.thickness << " m\n";
        std::cout << "  tip_force_z  : " << opt.tip_force_z << " N (total, distributed over "
                  << tip_nodes.size() << " nodes)\n";
        std::cout << "  schedule     : Fz=const for steps 0.." << (release_step - 1) << ", then Fz=0 for steps "
                  << release_step << ".." << (total_steps - 1) << "\n";
        std::cout << "  dt           : " << opt.step_size << " s\n";
        std::cout << "  steps        : " << total_steps << " (t_end=" << (total_steps * opt.step_size) << " s)\n\n";
    }

    if (opt.msglvl >= 2) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "time, avg_tip_z\n";
    }

    std::vector<std::pair<double, double>> csv_rows;
    if (opt.write_csv) {
        csv_rows.reserve(total_steps);
    }

    const auto wall_start = std::chrono::steady_clock::now();
    for (int step = 0; step < total_steps; step++) {
        system.DoStepDynamics(opt.step_size);

        if (opt.write_csv || opt.msglvl >= 2) {
            // Average z across all tip-edge nodes (matches GPU library metric)
            double avg_tip_z = 0.0;
            for (const auto& node : tip_nodes) {
                avg_tip_z += node->GetPos().z();
            }
            avg_tip_z /= static_cast<double>(tip_nodes.size());

            const double time = system.GetChTime();

            if (opt.write_csv) {
                csv_rows.emplace_back(time, avg_tip_z);
            }
            if (opt.msglvl >= 2) {
                std::cout << time << ", " << avg_tip_z << "\n";
            }
        }
    }
    const auto wall_end = std::chrono::steady_clock::now();

    const double wall_s = std::chrono::duration<double>(wall_end - wall_start).count();
    const double sim_s  = total_steps * opt.step_size;
    const double rtf    = (sim_s > 0) ? (wall_s / sim_s) : 0.0;

    if (opt.msglvl >= 1) {
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Wall time (loop)     : " << wall_s << " s\n";
        std::cout << "Simulation time      : " << sim_s  << " s\n";
        std::cout << "RTF (wall/sim)       : " << rtf    << "\n\n";
    }

    if (opt.write_csv) {
        std::ofstream of(opt.csv_filename);
        if (!of) {
            std::cerr << "ERROR: cannot open CSV file '" << opt.csv_filename << "' for writing\n";
        } else {
            of << "time,avg_tip_z\n";
            of << std::fixed << std::setprecision(12);
            for (const auto& [time, avg_tip_z] : csv_rows) {
                of << time << "," << avg_tip_z << "\n";
            }
            if (opt.msglvl >= 1) {
                std::cout << "Wrote CSV: " << opt.csv_filename << " (" << csv_rows.size() << " rows)\n\n";
            }
        }
    }

    // Final summary: average position of the tip edge
    if (opt.msglvl >= 1) {
        double avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
        for (const auto& node : tip_nodes) {
            avg_x += node->GetPos().x();
            avg_y += node->GetPos().y();
            avg_z += node->GetPos().z();
        }
        const double n = static_cast<double>(tip_nodes.size());
        avg_x /= n;
        avg_y /= n;
        avg_z /= n;
        std::cout << "Final avg tip pos    : (" << avg_x << ", " << avg_y << ", " << avg_z << ") m\n";
    }

    return 0;
}
