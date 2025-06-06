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
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkTSDA.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkTSDA)

// Perturbation for finite-difference Jacobian approximation
const double ChLinkTSDA::m_FD_delta = 1e-8;

ChLinkTSDA::ChLinkTSDA()
    : m_auto_rest_length(true),
      m_rest_length(0),
      m_length(0),
      m_length_dt(0),
      m_stiff(false),
      m_k(0),
      m_r(0),
      m_f(0),
      m_force_fun(nullptr),
      m_force(0),
      m_ode_fun(nullptr),
      m_nstates(0),
      m_variables(nullptr),
      m_jacobians(nullptr) {}

ChLinkTSDA::ChLinkTSDA(const ChLinkTSDA& other) : ChLink(other) {
    m_loc1 = other.m_loc1;
    m_loc2 = other.m_loc2;
    m_aloc1 = other.m_aloc1;
    m_aloc2 = other.m_aloc2;
    m_auto_rest_length = other.m_auto_rest_length;
    m_rest_length = other.m_rest_length;
    m_length = other.m_length;
    m_length_dt = other.m_length_dt;

    m_k = other.m_k;
    m_r = other.m_r;
    m_f = other.m_f;
    
    m_force = other.m_force;
    m_force_fun = other.m_force_fun;

    m_stiff = other.m_stiff;
    m_jacobians = nullptr;

    if (other.m_ode_fun) {
        RegisterODE(other.m_ode_fun);
    } else {
        m_ode_fun = nullptr;
        m_variables = nullptr;
        m_nstates = 0;
        m_Qforce.resize(12);
    }
}

ChLinkTSDA* ChLinkTSDA::Clone() const {
    return new ChLinkTSDA(*this);
}

ChLinkTSDA::~ChLinkTSDA() {
    delete m_variables;
    delete m_jacobians;
}

void ChLinkTSDA::RegisterODE(ODE* functor) {
    m_ode_fun = functor;
    m_nstates = functor->GetNumStates();
    m_states.resize(m_nstates);
    functor->SetInitialConditions(m_states, *this);
    m_variables = new ChVariablesGenericDiagonalMass(m_nstates);
    m_variables->GetMassDiagonal().Constant(m_nstates, 1);
    // Resize vector of forcing terms
    m_Qforce.resize(12 + m_nstates);
}

void ChLinkTSDA::Initialize(std::shared_ptr<ChBody> body1,
                            std::shared_ptr<ChBody> body2,
                            bool local,
                            const ChVector3d& loc1,
                            const ChVector3d& loc2) {
    m_body1 = (ChBodyFrame*)body1.get();
    m_body2 = (ChBodyFrame*)body2.get();

    if (local) {
        m_loc1 = loc1;
        m_loc2 = loc2;
        m_aloc1 = body1->TransformPointLocalToParent(loc1);
        m_aloc2 = body2->TransformPointLocalToParent(loc2);
    } else {
        m_loc1 = body1->TransformPointParentToLocal(loc1);
        m_loc2 = body2->TransformPointParentToLocal(loc2);
        m_aloc1 = loc1;
        m_aloc2 = loc2;
    }

    m_length = (m_aloc1 - m_aloc2).Length();
    if (m_auto_rest_length)
        m_rest_length = m_length;

    // Set size of forcing term
    m_Qforce.resize(12 + m_nstates);
}

void ChLinkTSDA::SetRestLength(double len) {
    m_auto_rest_length = false;
    m_rest_length = len;
}

// -----------------------------------------------------------------------------

void ChLinkTSDA::ComputeQ(double time,                  // current time
                          const ChState& state_x,       // state position to evaluate Q
                          const ChStateDelta& state_w,  // state speed to evaluate Q
                          ChVectorDynamic<>& Qforce     // output forcing vector
) {
    // Extract states and state derivatives for the two connected bodies
    ChFrameMoving<> bframe1;
    bframe1.SetCoordsys(state_x.segment(0, 7));
    bframe1.SetPosDt(state_w.segment(0, 3));
    bframe1.SetAngVelLocal(state_w.segment(3, 3));

    ChFrameMoving<> bframe2;
    bframe2.SetCoordsys(state_x.segment(7, 7));
    bframe2.SetPosDt(state_w.segment(6, 3));
    bframe2.SetAngVelLocal(state_w.segment(9, 3));

    // Extract internal ODE states
    if (m_variables) {
        m_states = state_w.segment(12, m_nstates);
    }

    // Calculate kinematics of the link
    m_aloc1 = bframe1.TransformPointLocalToParent(m_loc1);
    m_aloc2 = bframe2.TransformPointLocalToParent(m_loc2);

    ChVector3d avel1 = bframe1.PointSpeedLocalToParent(m_loc1);
    ChVector3d avel2 = bframe2.PointSpeedLocalToParent(m_loc2);

    ChVector3d dir = (m_aloc1 - m_aloc2).GetNormalized();
    m_length = (m_aloc1 - m_aloc2).Length();
    m_length_dt = Vdot(dir, avel1 - avel2);

    // Calculate force in the spring direction and convert to 3-D force.
    if (m_force_fun) {
        m_force = m_force_fun->evaluate(time, m_rest_length, m_length, m_length_dt, *this);
    } else {
        m_force = m_f - m_k * (m_length - m_rest_length) - m_r * m_length_dt;
    }
    ChVector3d Cforce = m_force * dir;

    // Load forcing terms acting on body1 (applied force is Cforce).
    auto atorque1 = Vcross(m_aloc1 - bframe1.GetPos(), Cforce);         // applied torque (absolute frame)
    auto ltorque1 = bframe1.TransformDirectionParentToLocal(atorque1);  // applied torque (local frame)
    Qforce.segment(0, 3) = Cforce.eigen();
    Qforce.segment(3, 3) = ltorque1.eigen();

    // Load forcing terms acting on body2 (applied force is -Cforce).
    auto atorque2 = Vcross(m_aloc2 - bframe2.GetPos(), -Cforce);        // applied torque (absolute frame)
    auto ltorque2 = bframe2.TransformDirectionParentToLocal(atorque2);  // applied torque (local frame)
    Qforce.segment(6, 3) = -Cforce.eigen();
    Qforce.segment(9, 3) = ltorque2.eigen();

    // Load ODE forcing term
    if (m_variables) {
        ChVectorDynamic<> rhs(m_nstates);
        m_ode_fun->CalculateRHS(time, m_states, rhs, *this);
        Qforce.segment(12, m_nstates) = rhs;
    }
}

void ChLinkTSDA::CreateJacobianMatrices() {
    m_jacobians = new SpringJacobians;

    // Collect all variables associated with this element
    std::vector<ChVariables*> variables_list;
    static_cast<ChBody*>(m_body1)->LoadableGetVariables(variables_list);
    static_cast<ChBody*>(m_body2)->LoadableGetVariables(variables_list);
    if (m_variables) {
        variables_list.push_back(m_variables);
    }

    // Set the associated variables in the underlying KRM block
    m_jacobians->m_KRM.SetVariables(variables_list);

    // Resize matrices
    m_jacobians->m_J.resize(m_nstates, m_nstates);
    m_jacobians->m_K.resize(12 + m_nstates, 12 + m_nstates);
    m_jacobians->m_R.resize(12 + m_nstates, 12 + m_nstates);
}

void ChLinkTSDA::ComputeJacobians(double time,                 // current time
                                  const ChState& state_x,      // state position to evaluate jacobians
                                  const ChStateDelta& state_w  // state speed to evaluate jacobians
) {
    ChVectorDynamic<> Qforce1(12 + m_nstates);  // forcing vector after perturbation
    ChVectorDynamic<> Jcolumn(12 + m_nstates);  // Jacobian column

    // Compute Jacobian w.r.t. states of the connected bodies (load first 12 columns in m_K matrix)
    // Note 1: that we must "bake in" the velocity transform: perturb at velocity level and let the body increment state
    // Note 2: the ODE internal states are not perturbed in this phase (so same as in state_x)
    ChState state_x_perturbed(14 + m_nstates, nullptr);
    state_x_perturbed.segment(14, m_nstates) = state_x.segment(14, m_nstates);

    ChStateDelta state_delta(12 + m_nstates, nullptr);
    state_delta.setZero(12 + m_nstates, nullptr);

    for (int i = 0; i < 12; i++) {
        state_delta(i) += m_FD_delta;
        static_cast<ChBody*>(m_body1)->LoadableStateIncrement(0, state_x_perturbed, state_x, 0, state_delta);
        static_cast<ChBody*>(m_body2)->LoadableStateIncrement(7, state_x_perturbed, state_x, 6, state_delta);
        ComputeQ(time, state_x_perturbed, state_w, Qforce1);
        state_delta(i) -= m_FD_delta;
        Jcolumn = (Qforce1 - m_Qforce) * (1 / m_FD_delta);
        m_jacobians->m_K.col(i) = Jcolumn;
    }

    // Compute Jacobian w.r.t. state derivatives of the connected bodies (load first 12 columns in m_R matrix)
    ChStateDelta& state_w_perturbed = const_cast<ChStateDelta&>(state_w);

    for (int i = 0; i < 12; i++) {
        state_w_perturbed(i) += m_FD_delta;
        ComputeQ(time, state_x, state_w_perturbed, Qforce1);
        state_w_perturbed(i) -= m_FD_delta;
        Jcolumn = (Qforce1 - m_Qforce) * (1 / m_FD_delta);
        m_jacobians->m_R.col(i) = Jcolumn;
    }

    // Compute Jacobian w.r.t. internal states (no contribution to m_K; load last nstates columns in m_R).
    if (m_variables) {
        m_jacobians->m_K.rightCols(m_nstates).setZero();

        for (int i = 0; i < m_nstates; i++) {
            state_w_perturbed(12 + i) += m_FD_delta;
            ComputeQ(time, state_x, state_w_perturbed, Qforce1);
            Jcolumn = (Qforce1 - m_Qforce) * (1 / m_FD_delta);
            m_jacobians->m_R.col(12 + i) = Jcolumn;
            state_w_perturbed(12 + i) -= m_FD_delta;
        }

        // Overwrite ODE Jacobian (rhs w.r.t internal states) if provided.
        m_jacobians->m_J.setZero();
        bool overwrite =
            m_ode_fun->CalculateJac(time, m_states, m_Qforce.segment(12, m_nstates), m_jacobians->m_J, *this);
        if (overwrite) {
            m_jacobians->m_R.bottomRightCorner(m_nstates, m_nstates) = m_jacobians->m_J;
        }
    }
}

// -----------------------------------------------------------------------------

void ChLinkTSDA::Update(double time, bool update_assets) {
    ChLink::Update(time, update_assets);

    // Pack states and state derivatives for the two connected bodies and ODE states (if present)
    ChState state_x(14 + m_nstates, nullptr);
    ChStateDelta state_w(12 + m_nstates, nullptr);

    static_cast<ChBody*>(m_body1)->LoadableGetStateBlockPosLevel(0, state_x);
    static_cast<ChBody*>(m_body2)->LoadableGetStateBlockPosLevel(7, state_x);

    static_cast<ChBody*>(m_body1)->LoadableGetStateBlockVelLevel(0, state_w);
    static_cast<ChBody*>(m_body2)->LoadableGetStateBlockVelLevel(6, state_w);

    if (m_variables) {
        state_x.segment(14, m_nstates).setZero();
        state_w.segment(12, m_nstates) = m_states;
    }

    // Compute forcing terms at current states (this also updates link kinematics)
    ComputeQ(time, state_x, state_w, m_Qforce);

    // Compute Jacobians (if needed)
    if (m_stiff) {
        if (!m_jacobians)
            CreateJacobianMatrices();
        // Compute Jacobian matrices
        ComputeJacobians(time, state_x, state_w);
        // Restore the states (may have been perturbed during Jacobian calculation)
        m_states = state_w.segment(12, m_nstates);
    }

    // TODO: DARIOM double check if correct
    ChVector3d dir = (m_aloc1 - m_aloc2).GetNormalized();
    react_force = -m_force * dir;
    react_torque = VNULL;
}

// -----------------------------------------------------------------------------

void ChLinkTSDA::InjectVariables(ChSystemDescriptor& descriptor) {
    // The base class does not actually inject any variables
    ChLink::InjectVariables(descriptor);

    if (m_variables) {
        m_variables->SetDisabled(!IsActive());
        descriptor.InsertVariables(m_variables);
    }
}

void ChLinkTSDA::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (m_jacobians) {
        descriptor.InsertKRMBlock(&m_jacobians->m_KRM);
    }
}

// -----------------------------------------------------------------------------

void ChLinkTSDA::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                ChState& x,                // state vector, position part
                                const unsigned int off_v,  // offset in v state vector
                                ChStateDelta& v,           // state vector, speed part
                                double& T                  // time
) {
    if (!IsActive())
        return;

    if (m_variables) {
        x.segment(off_x, m_nstates).setZero();
        v.segment(off_v, m_nstates) = m_states;
        T = GetChTime();
    }
}

void ChLinkTSDA::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                 const ChState& x,          // state vector, position part
                                 const unsigned int off_v,  // offset in v state vector
                                 const ChStateDelta& v,     // state vector, speed part
                                 const double T,            // time
                                 bool full_update           // perform complete update
) {
    if (!IsActive())
        return;

    // Important: set the internal states first, as they will be used in Update.
    if (m_variables) {
        m_states = v.segment(off_v, m_nstates);
    }

    Update(T, full_update);
}

void ChLinkTSDA::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    if (!IsActive())
        return;

    if (m_variables) {
        a.segment(off_a, m_nstates) = m_Qforce.segment(12, m_nstates);
    }
}

void ChLinkTSDA::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    // Nothing to do here
}

void ChLinkTSDA::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                   ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                   const double c           // a scaling factor
) {
    if (!IsActive())
        return;

    // Add forces to connected bodies (from the current vector of forcing terms)
    if (m_body1->Variables().IsActive()) {
        R.segment(m_body1->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(0, 3);
        R.segment(m_body1->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(3, 3);
    }
    if (m_body2->Variables().IsActive()) {
        R.segment(m_body2->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(6, 3);
        R.segment(m_body2->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(9, 3);
    }

    // Add forcing term for internal variables
    if (m_variables) {
        R.segment(off, m_nstates) += c * m_Qforce.segment(12, m_nstates);
    }
}

void ChLinkTSDA::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                    ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& v,  // the v vector
                                    const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    if (m_variables) {
        R.segment(off, m_nstates) += c * v.segment(off, m_nstates);
    }
}

void ChLinkTSDA::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    if (!IsActive())
        return;

    if (m_variables) {
        Md.segment(off, m_nstates).array() += 1.0;
    }
}

void ChLinkTSDA::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,  // offset in L, Qc
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    if (m_variables) {
        m_variables->State() = v.segment(off_v, m_nstates);
        m_variables->Force() = R.segment(off_v, m_nstates);
    }
}

void ChLinkTSDA::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                   ChStateDelta& v,
                                   const unsigned int off_L,  // offset in L
                                   ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    if (m_variables) {
        v.segment(off_v, m_nstates) = m_variables->State();
    }
}

// -----------------------------------------------------------------------------

void ChLinkTSDA::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (m_jacobians) {
        // Recall to flip sign to load K = -dQ/dx and R = -dQ/dv
        m_jacobians->m_KRM.GetMatrix() = -Kfactor * m_jacobians->m_K - Rfactor * m_jacobians->m_R;
    }
}

// -----------------------------------------------------------------------------

void ChLinkTSDA::VariablesFbReset() {
    if (m_variables) {
        m_variables->Force().setZero();
    }
}

void ChLinkTSDA::VariablesFbLoadForces(double factor) {
    if (m_variables) {
        m_variables->Force() = m_Qforce.segment(12, m_nstates);
    }
}

void ChLinkTSDA::VariablesQbLoadSpeed() {
    if (m_variables) {
        m_variables->State() = m_states;
    }
}

void ChLinkTSDA::VariablesQbSetSpeed(double step) {
    if (m_variables) {
        m_states = m_variables->State();
    }
}

void ChLinkTSDA::VariablesFbIncrementMq() {
    if (m_variables) {
        m_variables->AddMassTimesVector(m_variables->Force(), m_variables->State());
    }
}

void ChLinkTSDA::VariablesQbIncrementPosition(double dt_step) {
    //// RADU: correct?
    // nothing to do here
}

void ChLinkTSDA::ConstraintsFbLoadForces(double factor) {
    if (!IsActive())
        return;

    // Add forces to connected bodies (from the current vector of forcing terms)
    m_body1->Variables().Force().segment(0, 3) += factor * m_Qforce.segment(0, 3);
    m_body1->Variables().Force().segment(3, 3) += factor * m_Qforce.segment(3, 3);

    m_body2->Variables().Force().segment(0, 3) += factor * m_Qforce.segment(6, 3);
    m_body2->Variables().Force().segment(3, 3) += factor * m_Qforce.segment(9, 3);
}

// -----------------------------------------------------------------------------

void ChLinkTSDA::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkTSDA>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_loc1);
    archive_out << CHNVP(m_loc2);
    archive_out << CHNVP(m_aloc1);
    archive_out << CHNVP(m_aloc2);
    archive_out << CHNVP(m_auto_rest_length);
    archive_out << CHNVP(m_rest_length);
    archive_out << CHNVP(m_length);
    archive_out << CHNVP(m_length_dt);
    archive_out << CHNVP(m_stiff);
    archive_out << CHNVP(m_k);
    archive_out << CHNVP(m_r);
    archive_out << CHNVP(m_f);
}

void ChLinkTSDA::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkTSDA>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_loc1);
    archive_in >> CHNVP(m_loc2);
    archive_in >> CHNVP(m_aloc1);
    archive_in >> CHNVP(m_aloc2);
    archive_in >> CHNVP(m_auto_rest_length);
    archive_in >> CHNVP(m_rest_length);
    archive_in >> CHNVP(m_length);
    archive_in >> CHNVP(m_length_dt);
    archive_in >> CHNVP(m_stiff);
    archive_in >> CHNVP(m_k);
    archive_in >> CHNVP(m_r);
    archive_in >> CHNVP(m_f);
}

}  // end namespace chrono
