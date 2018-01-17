SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(ubitrack_device_camera_msmf)

cmake_minimum_required(VERSION 3.4.0)

include(CheckIncludeFile)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(KEEP_RPATHS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)


# External Libraries (could this be solved with conan dependencies ??)

# OpenGL
SET(HAVE_OPENGL 0)
find_package(OpenGL)
IF(OpenGL_FOUND)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_OPENGL")
  SET(HAVE_OPENGL 1)
  MESSAGE(STATUS "Found OpenGL: ${OpenGL_INCLUDE_DIR} - ${OpenGL_LIBRARIES}")
ENDIF(OpenGL_FOUND)

# OpenCL
SET(HAVE_OPENCL 0)
find_package(OpenCL)
IF(OpenCL_FOUND)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_OPENCL")
  include_directories(${OpenCL_INCLUDE_DIR})
  SET(HAVE_OPENCL 1)
  MESSAGE(STATUS "Found OpenCL: ${OpenCL_INCLUDE_DIR} - ${OpenCL_LIBRARY}")
ENDIF(OpenCL_FOUND)

set(UBITRACK_EXTRA_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${OpenCL_LIBRARY})


IF(WIN32)
	FIND_PACKAGE(WindowsSDK)
	if(WINDOWSSDK_FOUND)
		get_windowssdk_include_dirs(${WINDOWSSDK_LATEST_DIR} WINDOWSSDK_INCLUDE_DIR)
		# Look for the header file.
		FIND_PATH(MSMF_INCLUDE_DIR NAMES Mfapi.h
		                              PATHS ${WINDOWSSDK_INCLUDE_DIR}
		                              DOC "Path in which the file Mfapi.h is located." )
		MESSAGE(STATUS "Found Windows SDK: ${WINDOWSSDK_LATEST_NAME} -> ${MSMF_INCLUDE_DIR}")
		include_directories(${MSMF_INCLUDE_DIR})
		set(WITH_MSMF ON)
	endif(WINDOWSSDK_FOUND)

	unset(HAVE_MSMF CACHE)
	if(WITH_MSMF)
	  check_include_file(Mfapi.h HAVE_MSMF)
	endif(WITH_MSMF)

	if(HAVE_MSMF)
		set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_MSMF")

		add_subdirectory(src/MSMFFrameGrabber)

		# collect all utql patterns from module for installation
		file(GLOB _utql_patterns "doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
		foreach(pfile ${_utql_patterns})
		  string(REGEX REPLACE "^.*/doc/utql/" "" pfile2 "${pfile}")
		  GET_FILENAME_COMPONENT(fpath ${pfile2} PATH)
		  IF(fpath)
			install(FILES ${pfile} DESTINATION "${UBITRACK_UTQLPATTERN_INSTALL_DIRECTORY}/${fpath}" COMPONENT data)
		  ELSE(fpath)
		    install(FILES ${pfile} DESTINATION "${UBITRACK_UTQLPATTERN_INSTALL_DIRECTORY}" COMPONENT data)
		  ENDIF(fpath)
		endforeach()
	endif(HAVE_MSMF)
ENDIF(WIN32)
