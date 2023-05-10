# Install script for directory: /home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xlibassimp5.2.4-devx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/build_assimp/lib/Debug/libassimp_d.a")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/build_assimp/lib/Release/libassimp.a")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/build_assimp/lib/RelWithDebInfo/libassimp_rd.a")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xassimp-devx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/assimp" TYPE FILE FILES
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/anim.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/aabb.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/ai_assert.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/camera.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/color4.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/color4.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/build_assimp/code/../include/assimp/config.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/ColladaMetaData.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/commonMetaData.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/defs.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/cfileio.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/light.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/material.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/material.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/matrix3x3.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/matrix3x3.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/matrix4x4.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/matrix4x4.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/mesh.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/ObjMaterial.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/pbrmaterial.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/GltfMaterial.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/postprocess.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/quaternion.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/quaternion.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/scene.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/metadata.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/texture.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/types.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/vector2.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/vector2.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/vector3.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/vector3.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/version.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/cimport.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/importerdesc.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Importer.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/DefaultLogger.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/ProgressHandler.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/IOStream.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/IOSystem.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Logger.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/LogStream.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/NullLogger.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/cexport.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Exporter.hpp"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/DefaultIOStream.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/DefaultIOSystem.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/ZipArchiveIOSystem.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/SceneCombiner.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/fast_atof.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/qnan.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/BaseImporter.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Hash.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/MemoryIOWrapper.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/ParsingUtils.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/StreamReader.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/StreamWriter.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/StringComparison.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/StringUtils.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/SGSpatialSort.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/GenericProperty.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/SpatialSort.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/SkeletonMeshBuilder.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/SmallVector.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/SmoothingGroups.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/SmoothingGroups.inl"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/StandardShapes.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/RemoveComments.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Subdivision.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Vertex.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/LineSplitter.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/TinyFormatter.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Profiler.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/LogAux.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Bitmap.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/XMLTools.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/IOStreamBuffer.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/CreateAnimMesh.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/XmlParser.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/BlobIOSystem.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/MathFunctions.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Exceptional.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/ByteSwapper.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Base64.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xassimp-devx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/assimp/Compiler" TYPE FILE FILES
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Compiler/pushpack1.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Compiler/poppack1.h"
    "/home/jason/Desktop/STUDY/main_fork/chrono/contrib/build-scripts/vsg/download_vsg/assimp/code/../include/assimp/Compiler/pstdint.h"
    )
endif()

