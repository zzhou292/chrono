%{
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;
%}

%shared_ptr(chrono::vsg3d::ChVisualSystemVSG)

%import(module = "pychrono.core") "chrono_swig/interface/core/ChVisualSystem.i"

%include "../../../chrono_vsg/ChVisualSystemVSG.h"

%DefSharedPtrDynamicDowncast2NS(chrono, chrono::vsg3d, ChVisualSystem, ChVisualSystemVSG)
