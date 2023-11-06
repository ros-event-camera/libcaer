# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

# Project name and version
PROJECT(libcaer VERSION 3.3.15 LANGUAGES C CXX)

# Basic setup, useful variables, see docs.
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmakemod ${CMAKE_MODULE_PATH})
SET(ENABLE_ALL_WARNINGS ON CACHE BOOL "Turn on all warnings for build" FORCE)
INCLUDE(inivation-setup)

# Legacy cmake support.
INSTALL(FILES cmakemod/caer-base.cmake DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer)

# Override default settings
SET(CMAKE_CXX_STANDARD 11)

OPTION(ENABLE_STATIC "Build and install static library in addition to default shared library" OFF)
OPTION(ENABLE_SERIALDEV "Enable support for serial port devices using libserialport" OFF)
OPTION(ENABLE_OPENCV "Enable support for frame enhancements using OpenCV" OFF)
OPTION(UDEV_INSTALL "Install udev rules on Linux" ON)
OPTION(EXAMPLES_INSTALL "Build and install examples" OFF)
OPTION(BUILD_CONFIG_VCPKG "Set build environment compatible with VCPKG" OFF)
OPTION(BUILD_SHARED_LIBS "Build libcaer as a shared library" ON)

# Cross-compile support
IF(NOT USER_LOCAL_PREFIX)
	SET(USER_LOCAL_PREFIX ${CMAKE_INSTALL_PREFIX})
ENDIF()

# Required: threads, math
IF(NOT CC_MSVC)
	SET(BASE_LIBS m)
	SET(LIBCAER_PKGCONFIG_LIBS_PRIVATE "-lm")
ENDIF()

IF(CC_MSVC)
	INCLUDE_DIRECTORIES(thirdparty/simple-stdatomic/)
ENDIF()

SET(BASE_LIBS ${BASE_LIBS} ${SYSTEM_THREAD_LIBS})
SET(LIBCAER_PKGCONFIG_LIBS_PRIVATE "${LIBCAER_PKGCONFIG_LIBS_PRIVATE} ${SYSTEM_THREAD_LIBS}")

# Windows needs extra winsock library for portable endian functions.
IF(OS_WINDOWS)
	SET(BASE_LIBS ${BASE_LIBS} ws2_32)
ENDIF()

MESSAGE(STATUS "Base libraries: ${BASE_LIBS}")

# Search for external libraries with pkg-config
INCLUDE(FindPkgConfig)

# Required: basic USB devices support
IF(CC_MSVC)
	FIND_PACKAGE(libusb CONFIG REQUIRED)
ELSE()
	PKG_CHECK_MODULES(
		libusb
		REQUIRED
		IMPORTED_TARGET
		libusb-1.0>=1.0.17)
	SET(LIBCAER_PKGCONFIG_REQUIRES_PRIVATE "libusb-1.0 >= 1.0.17")
ENDIF()

# Optional: serial devices support
IF(ENABLE_SERIALDEV)
	# Require libserialport, minimum 0.1.1 version.
	PKG_CHECK_MODULES(
		libserialport
		REQUIRED
		IMPORTED_TARGET
		libserialport>=0.1.1)
	SET(LIBCAER_PKGCONFIG_REQUIRES_PRIVATE "${LIBCAER_PKGCONFIG_REQUIRES_PRIVATE}, libserialport >= 0.1.1")
ENDIF()

# Optional: OpenCV support for frame enhancement
IF(ENABLE_OPENCV)
	# OpenCV support.
	FIND_PACKAGE(OpenCV REQUIRED COMPONENTS core imgproc)
	IF(OpenCV_VERSION VERSION_LESS "3.1.0")
		MESSAGE(FATAL_ERROR "Cannot find OpenCV 3.1.0 or newer.")
	ENDIF()
ENDIF()

# Add main include dir for all targets
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/)

# Subdirectories
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)

# Generate pkg-config file
CONFIGURE_FILE(libcaer.pc.in libcaer.pc @ONLY)

INSTALL(FILES ${CMAKE_BINARY_DIR}/libcaer.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Export the cmake configuration for the install tree
INCLUDE(CMakePackageConfigHelpers)
WRITE_BASIC_CONFIG_VERSION_FILE(${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake VERSION ${PROJECT_VERSION}
								COMPATIBILITY SameMajorVersion)
SET(include_dirs ${CMAKE_INSTALL_INCLUDEDIR})
IF(BUILD_CONFIG_VCPKG)
	SET(export_destination ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
ELSE()
	SET(export_destination ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
ENDIF()
CONFIGURE_PACKAGE_CONFIG_FILE(
	${PROJECT_SOURCE_DIR}/cmakemod/${PROJECT_NAME}Config.cmake.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	INSTALL_DESTINATION ${export_destination} PATH_VARS include_dirs export_destination)
INSTALL(EXPORT libcaer-exports NAMESPACE libcaer:: DESTINATION ${export_destination})
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
		DESTINATION ${export_destination})
UNSET(include_dirs)
UNSET(export_destination)

# Install udev rules
IF(OS_LINUX AND UDEV_INSTALL)
	MESSAGE(STATUS "Installing global udev rules into ${CMAKE_INSTALL_PREFIX}/lib/udev/rules.d/")

	INSTALL(FILES docs/65-inivation.rules DESTINATION lib/udev/rules.d/)
ENDIF()

# Compile all examples
IF(EXAMPLES_INSTALL)
	ADD_SUBDIRECTORY(examples)
ENDIF()

# Support automatic RPM generation
SET(CPACK_PACKAGE_NAME ${PROJECT_NAME})
SET(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
SET(CPACK_PACKAGE_RELEASE 1)
SET(CPACK_PACKAGE_CONTACT "support@inivation.com")
SET(CPACK_PACKAGE_VENDOR "iniVation AG")
SET(CPACK_PACKAGE_DESCRIPTION
	"Minimal C library to access, configure and get/send AER data from sensors or to/from neuromorphic processors.")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${CPACK_PACKAGE_DESCRIPTION})
SET(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}.${CMAKE_SYSTEM_PROCESSOR}")
SET(CPACK_GENERATOR "RPM" "DEB")
SET(CPACK_RPM_PACKAGE_AUTOREQ 1)
# SET(CPACK_RPM_PACKAGE_REQUIRES "libusbx >= 1.0.17, libserialport >= 0.1.1, opencv >= 3.1.0")
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS 1)
# SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libusb-1.0-0 (>= 1.0.17), libserialport0 (>= 0.1.1), libopencv-contrib3.1 (>=
# 3.1.0)")

INCLUDE(CPack)
