set(UBITRACK_LIBRARY_NAME device_camera_msmf)

# All cpp files for utvisualization components
FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(${UBITRACK_LIBRARY_NAME} SHARED ${UBITRACK_SRCS})

target_link_libraries(${UBITRACK_LIBRARY_NAME} 
    CONAN_PKG::ubitrack_core
    CONAN_PKG::ubitrack_vision
    CONAN_PKG::ubitrack_dataflow
    OpenGL::GL
    OpenGL::GLU
    OpenCL::OpenCL
     ${UBITRACK_EXTRA_LIBS}
    )

target_include_directories(${UBITRACK_LIBRARY_NAME} PRIVATE
  ${MSMF_INCLUDE_DIR}
  )

# do not create symlinks to versioned names of component libraries
set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES NO_SONAME ON)

# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
endif()

ubitrack_set_target_properties(${UBITRACK_LIBRARY_NAME})
ubitrack_install_component(${UBITRACK_LIBRARY_NAME})