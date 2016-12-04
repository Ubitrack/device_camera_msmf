IF(WIN32)

	set(the_description "The UbiTrack MSMF Components")
	ut_add_component(device_camera_msmf DEPS utcore utdataflow utvision )
	

	SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

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

	ut_clear_vars(HAVE_MSMF)
	if(WITH_MSMF)
	  check_include_file(Mfapi.h HAVE_MSMF)
	endif(WITH_MSMF)

	if(HAVE_MSMF)
		add_definitions(-DHAVE_MSMF)
		ut_component_include_directories("src/MSMFFrameGrabber" ${TINYXML_INCLUDE_DIR} ${LOG4CPP_INCLUDE_DIR} ${BOOSTBINDINGS_INCLUDE_DIR} ${LAPACK_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} ${MSMF_INCLUDE_DIR})
		ut_glob_component_sources(HEADERS "src/MSMFFrameGrabber/*.hpp" SOURCES "src/MSMFFrameGrabber/*.cpp")
		ut_create_single_component()
		ut_install_utql_patterns()
	endif(HAVE_MSMF)

ENDIF(WIN32)