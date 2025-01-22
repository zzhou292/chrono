# ------------------------------------------------------------------------------
# List of demos according their dependencies
# ------------------------------------------------------------------------------

set(DEMOS
    demo_SYN_DDS_distributed
)

if(ENABLE_MODULE_IRRLICHT OR ENABLE_MODULE_VSG)
   set(DEMOS ${DEMOS}
       demo_SYN_DDS_wheeled 
       demo_SYN_DDS_wheeled_collision
       demo_SYN_DDS_tracked
       demo_SYN_DDS_tracked_collision
       demo_SYN_DDS_scm
       jz_SYN_DDS_viper
       jz_SYN_DDS_viper_smc
       jz_SYN_DDS_curiosity
       jz_SYN_DDS_roboenv
       jz2_SYN_DDS_drop
       jz2_SYN_DDS_drop_2
       jz2_SYN_DDS_drop_2_whole
   )
endif()

# --------------------------------------------------------------

find_package(fastrtps REQUIRED)

include_directories(${SYN_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${SYN_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoEngine_synchrono")
list(APPEND LIBS "ChronoModels_vehicle")

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHT_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
    if(ENABLE_MODULE_VEHICLE) 
      list(APPEND LIBS "ChronoEngine_vehicle_vsg")
    endif()
endif()
foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  source_group("" FILES ${PROGRAM}.cpp)

  set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  
  target_link_libraries(${PROGRAM} ${LIBS})

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
