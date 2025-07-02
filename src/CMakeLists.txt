add_library(ModifiedTasks SHARED ModifiedTasks.cpp ModifiedTasks.h)

target_include_directories(ModifiedTasks PUBLIC 
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${MC_RTC_INCLUDE_DIR}>
)

target_link_libraries(ModifiedTasks PUBLIC mc_rtc::mc_tasks)

install(
    TARGETS ModifiedTasks
    EXPORT "${TARGETS_EXPORT_NAME}"
    RUNTIME DESTINATION "${MC_RTC_BINDIR}"
    LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
    ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  )

set(controller_SRC
  SbsController.cpp
)

set(controller_HDR
  SbsController.h
  
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
target_link_libraries(${PROJECT_NAME} PRIVATE ModifiedTasks)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSbsController_EXPORTS")
