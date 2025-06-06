#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

message(STATUS "SPH-based Chrono::FSI submodule")

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message(WARNING "Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::FSI")
    set(CH_ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

# Return now if CUDA is not available
if(NOT CHRONO_CUDA_FOUND)
    message(WARNING "Chrono::FSI::SPH requires CUDA, but CUDA was not found; disabling Chrono::FSI::SPH")
    set(CH_ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

message(STATUS "  Chrono CUDA architectures: ${CHRONO_CUDA_ARCHITECTURES}")

#-----------------------------------------------------------------------------
# List all files for the SPH-based Chrono::FSI library
#-----------------------------------------------------------------------------

# --------------- SPH FSI FILES

set(FSISPH_BASE_FILES
    ChFsiDataTypesSPH.h
    ChFsiDefinitionsSPH.h
    ChFsiParamsSPH.h
    ChFsiProblemSPH.h
    ChFsiSystemSPH.h
    ChFsiInterfaceSPH.h
    ChFsiFluidSystemSPH.h
    ChFsiPrintUtilsSPH.h

    ChFsiProblemSPH.cpp
    ChFsiSystemSPH.cpp
    ChFsiInterfaceSPH.cpp
    ChFsiFluidSystemSPH.cpp
)
source_group("" FILES ${FSISPH_BASE_FILES})

set(FSISPH_PHYSICS_FILES
    physics/MarkerType.cuh
    physics/FsiDataManager.cuh
    physics/BceManager.cuh
    physics/FluidDynamics.cuh
    physics/CollisionSystem.cuh
    physics/FsiForce.cuh    
    physics/FsiForceWCSPH.cuh
    physics/FsiForceISPH.cuh
    physics/SphGeneral.cuh

    physics/FsiDataManager.cu
 	physics/BceManager.cu
    physics/FluidDynamics.cu
    physics/CollisionSystem.cu
    physics/FsiForce.cu
    physics/FsiForceWCSPH.cu
    physics/FsiForceISPH.cu
    physics/SphGeneral.cu
)
source_group("physics" FILES ${FSISPH_PHYSICS_FILES})

set(FSISPH_MATH_FILES
    math/LinearSolver.h
    #math/LinearSolverBiCGStab.h
    #math/LinearSolverGMRES.h

    math/CustomMath.cuh
    math/ExactLinearSolvers.cuh

    #math/LinearSolverBiCGStab.cpp
    #math/LinearSolverGMRES.cpp
)
source_group("math" FILES ${FSISPH_MATH_FILES})

set(FSISPH_UTILS_FILES
    utils/UtilsPrintSph.cuh
    utils/UtilsDevice.cuh
    utils/UtilsTypeConvert.cuh

    utils/UtilsPrintSph.cu
    utils/UtilsDevice.cu
)
source_group("utils" FILES ${FSISPH_UTILS_FILES})

# --------------- 3rd party STB FILES

set(FSISPH_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)
source_group("stb" FILES ${FSISPH_STB_FILES})

# --------------- SPH FSI VSG INTERFACE

if(CH_ENABLE_MODULE_VSG)
    set(FSISPH_VSG_FILES
        visualization/ChFsiVisualizationVSG.h
        visualization/ChFsiVisualizationVSG.cpp
    )
else()
    set(FSISPH_VSG_FILES "")
endif()
source_group("" FILES ${FSISPH_VSG_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_FSI "")
if(CH_ENABLE_MODULE_VSG)
  list (APPEND DEPENDENCIES_FSI "VSG")
endif()

set(DEPENDENCIES_FSI ${DEPENDENCIES_FSI} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Create the Chrono_fsisph library
#-----------------------------------------------------------------------------

message(STATUS "  Add Chrono_fsisph library")

add_library(Chrono_fsisph
    ${FSISPH_BASE_FILES}
    ${FSISPH_PHYSICS_FILES}
    ${FSISPH_MATH_FILES}
    ${FSISPH_UTILS_FILES}
    ${FSISPH_STB_FILES}
)
add_library(Chrono::fsisph ALIAS Chrono_fsisph)

if (CH_STATIC)
  set_target_properties(Chrono_fsisph PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)              # missing DLL interface
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4100>)              # unreferenced formal parameter
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4324>)  # unreferenced formal parameter
  set_target_properties(Chrono_fsisph PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_fsisph PRIVATE Chrono_core)
target_link_libraries(Chrono_fsisph PUBLIC Chrono_fsi)

target_link_libraries(Chrono_fsisph PUBLIC CUDA::cudart_static)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::nvrtc)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::cuda_driver)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::cublas)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::cusparse)

target_link_libraries(Chrono_fsisph PUBLIC Thrust::Thrust)
target_compile_definitions(Chrono_fsisph PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
target_compile_definitions(Chrono_fsisph PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

target_compile_definitions(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)

set_target_properties(Chrono_fsisph PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})

install(TARGETS Chrono_fsisph
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_fsi)

#-------------------------------------------------------------------------------
# Add the Chrono_fsisph_vsg library
#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)
    message(STATUS "  Add Chrono_fsisph_vsg library")

    add_library(Chrono_fsisph_vsg
        ${FSISPH_VSG_FILES}
    )
    add_library(Chrono::fsisph_vsg ALIAS Chrono_fsisph_vsg)
   
    if (CH_STATIC)
      set_target_properties(Chrono_fsisph_vsg PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()
 
    if(MSVC)
      # disable warnings related to missing DLL interface
      target_compile_options(Chrono_fsisph_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
      set_target_properties(Chrono_fsisph_vsg PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_compile_definitions(Chrono_fsisph_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)
    
    target_link_libraries(Chrono_fsisph_vsg PRIVATE Chrono_core Chrono_fsisph)
    target_link_libraries(Chrono_fsisph_vsg PUBLIC Chrono_vsg)

    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cudart_static)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::nvrtc)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cuda_driver)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cublas)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cusparse)

    target_link_libraries(Chrono_fsisph_vsg PRIVATE Thrust::Thrust)
    target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
    target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

    install(TARGETS Chrono_fsisph_vsg
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_fsi) # TODO DARIOM: verify if correct folder

endif()
