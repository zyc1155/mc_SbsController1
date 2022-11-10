set(controller_SRC
  SbsController.cpp
)

set(controller_HDR
  SbsController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSbsController_EXPORTS")
