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

# Use same object files for both shared and static.
ADD_LIBRARY(libcaer OBJECT ${LIBCAER_SRC_FILES})

# Shared libraries need PIC enabled.
SET_TARGET_PROPERTIES(libcaer PROPERTIES POSITION_INDEPENDENT_CODE 1)

# This is often the case due to how we try to have multiple compatible
# definitions of functions in C.
TARGET_COMPILE_OPTIONS(libcaer PRIVATE -Wno-unused-function)

TARGET_LINK_LIBRARIES(libcaer PRIVATE PkgConfig::libusb ${BASE_LIBS})
TARGET_INCLUDE_DIRECTORIES(libcaer PRIVATE PkgConfig::libusb ${CMAKE_SOURCE_DIR}/include/)

IF(OS_LINUX)
	# Raspberry Pi support available only on Linux.
	TARGET_SOURCES(libcaer PRIVATE davis_rpi.c)
ENDIF()

IF (ENABLE_SERIALDEV)
	# Add serial devices.
	TARGET_SOURCES(libcaer PRIVATE edvs.c)

	TARGET_LINK_LIBRARIES(libcaer PRIVATE PkgConfig::libserialport)
	TARGET_INCLUDE_DIRECTORIES(libcaer PRIVATE PkgConfig::libserialport)
ENDIF()

IF (ENABLE_OPENCV)
	# Add C++ OpenCV support.
	TARGET_SOURCES(libcaer PRIVATE frame_utils_opencv.cpp)

	TARGET_LINK_LIBRARIES(libcaer PUBLIC ${OpenCV_LIBS})
	TARGET_INCLUDE_DIRECTORIES(libcaer PUBLIC ${OpenCV_INCLUDE_DIRS})

	# The included C headers need to do this (NULL is 0, nullptr not available).
	TARGET_COMPILE_OPTIONS(libcaer PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-zero-as-null-pointer-constant>)

	# Clang weirdness: also looks into the C code, which of course
	# will use old-style casts (the only cast available there).
	IF (CC_CLANG)
		TARGET_COMPILE_OPTIONS(libcaer PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-old-style-cast>)
	ENDIF()
ENDIF()

# Set full RPATH
SET(CMAKE_INSTALL_RPATH ${USER_LOCAL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

ADD_LIBRARY(caer SHARED $<TARGET_OBJECTS:libcaer>)

SET_TARGET_PROPERTIES(caer
	PROPERTIES
	SOVERSION ${PROJECT_VERSION_MAJOR}
	VERSION ${PROJECT_VERSION}
)

IF (OS_WINDOWS)
	INSTALL(TARGETS caer EXPORT libcaer-exports RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
ELSE()
	INSTALL(TARGETS caer EXPORT libcaer-exports DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF()

IF (ENABLE_STATIC)
	ADD_LIBRARY(caerStatic STATIC $<TARGET_OBJECTS:libcaer>)

	SET_TARGET_PROPERTIES(caerStatic
		PROPERTIES
		OUTPUT_NAME caer
	)

	IF (OS_WINDOWS)
		INSTALL(TARGETS caerStatic EXPORT libcaer-exports RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
	ELSE()
		INSTALL(TARGETS caerStatic EXPORT libcaer-exports DESTINATION ${CMAKE_INSTALL_LIBDIR})
	ENDIF()
ENDIF()
