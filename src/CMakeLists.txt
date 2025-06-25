set(controller_SRC
  SbsController.cpp
  ModifiedTasks.cpp
)

set(controller_HDR
  SbsController.h
  ModifiedTasks.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSbsController_EXPORTS")
