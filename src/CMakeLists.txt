# This is often the case due to how we try to have multiple compatible
# definitions of functions in C.
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")

SET(LIBCAER_SRC_FILES
	ringbuffer.c
	log.c
	frame_utils.c
	filters_dvs_noise.c
	usb_utils.c
	autoexposure.c
	device_discover.c
	device.c
	dvs128.c
	davis.c
	dynapse.c
	dvs132s.c)

IF(OS_LINUX)
	# Raspberry Pi support available only on Linux.
	SET(LIBCAER_SRC_FILES ${LIBCAER_SRC_FILES} davis_rpi.c)
ENDIF()

IF (ENABLE_SERIALDEV)
	# Add serial devices.
	SET(LIBCAER_SRC_FILES ${LIBCAER_SRC_FILES} edvs.c)
ENDIF()

IF (ENABLE_OPENCV)
	# Add C++ OpenCV support..
	SET(LIBCAER_SRC_FILES ${LIBCAER_SRC_FILES} frame_utils_opencv.cpp)

	# The included C headers need to do this (NULL is 0, nullptr not available).
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-zero-as-null-pointer-constant")

	# Clang weirdness: also looks into the C code, which of course
	# will use old-style casts (the only cast available there).
	IF (CC_CLANG)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-old-style-cast")
	ENDIF()
ENDIF()

# Set full RPATH
SET(CMAKE_INSTALL_RPATH ${USER_LOCAL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

ADD_LIBRARY(caer SHARED ${LIBCAER_SRC_FILES})

SET_TARGET_PROPERTIES(caer
	PROPERTIES
	SOVERSION ${PROJECT_VERSION_MAJOR}
	VERSION ${PROJECT_VERSION}
)

TARGET_LINK_LIBRARIES(caer PUBLIC ${LIBCAER_LIBS})
TARGET_INCLUDE_DIRECTORIES(caer PRIVATE ${LIBCAER_INCDIRS} ${CMAKE_SOURCE_DIR}/include/)
TARGET_INCLUDE_DIRECTORIES(caer PUBLIC ${OpenCV_INCLUDE_DIRS}) # Used in headers.

IF (OS_WINDOWS)
	INSTALL(TARGETS caer EXPORT libcaer-exports RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
ELSE()
	INSTALL(TARGETS caer EXPORT libcaer-exports DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF()

IF (ENABLE_STATIC)
	ADD_LIBRARY(caerStatic STATIC ${LIBCAER_SRC_FILES})

	SET_TARGET_PROPERTIES(caerStatic
		PROPERTIES
		OUTPUT_NAME caer
	)

	TARGET_LINK_LIBRARIES(caerStatic PUBLIC ${LIBCAER_LIBS})
	TARGET_INCLUDE_DIRECTORIES(caerStatic PRIVATE ${LIBCAER_INCDIRS} ${CMAKE_SOURCE_DIR}/include/)
	TARGET_INCLUDE_DIRECTORIES(caerStatic PUBLIC ${OpenCV_INCLUDE_DIRS}) # Used in headers.

	IF (OS_WINDOWS)
		INSTALL(TARGETS caerStatic EXPORT libcaer-exports RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
	ELSE()
		INSTALL(TARGETS caerStatic EXPORT libcaer-exports DESTINATION ${CMAKE_INSTALL_LIBDIR})
	ENDIF()
ENDIF()
