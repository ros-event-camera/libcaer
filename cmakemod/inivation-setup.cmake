# Execute various useful checks and setup steps: - setup global build settings (CMAKE_BUILD_TYPE, CMAKE_INSTALL_PREFIX)
# and provide support for Clang's libc++ via USE_CLANG_LIBCPP - setup C11 and C++20 requirements, prevent symbol leakage
# - define proper installation directories via GNUInstallDirs - enable compiler cache (ccache) - verify that size of
# void* and size_t agree (useful for low-level operations) - add useful definitions for compilation (XOPEN_SOURCE=700,
# DEFAULT_SOURCE=1, DARWIN_C_SOURCE=1, FILE_OFFSET_BITS=64, USE_MINGW_ANSI_STDIO=1) as needed - enable all compiler
# warnings for GCC and Clang - setup RPATH correctly - setup linking to only link to libraries with required symbols -
# print info variables for debug purposes
#
# And set several useful variables for common system information: - detect compiler (CC_CLANG, CC_GCC, CC_ICC, CC_MSVC)
# - detect operating system (OS_UNIX, OS_LINUX, OS_MACOS, OS_WINDOWS) - detect if system big/little-endian in
# SYSTEM_BIGENDIAN - detect thread libraries (HAVE_PTHREADS, HAVE_WIN32_THREADS) and define libs needed for successful
# linking in SYSTEM_THREAD_LIBS

# General build settings
IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Possible build types: None Debug Release RelWithDebInfo MinSizeRel"
										 FORCE)
	# Set the possible values of build type for cmake-gui
	SET_PROPERTY(
		CACHE CMAKE_BUILD_TYPE
		PROPERTY STRINGS
				 "Debug"
				 "Release"
				 "MinSizeRel"
				 "RelWithDebInfo")
ENDIF()

IF(NOT CMAKE_INSTALL_PREFIX)
	IF(UNIX AND NOT APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake default install prefix, set to /usr on Unix/Linux" FORCE)
	ELSEIF(APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr/local" CACHE STRING "CMake default install prefix, set to /usr/local on macOS"
													FORCE)
	ELSE()
		MESSAGE(FATAL_ERROR "CMAKE_INSTALL_PREFIX is not set")
	ENDIF()
ENDIF()

# Extra options.
OPTION(USE_CLANG_LIBCPP "Use and link to LLVM's libc++ instead of GCC's libstdc++." OFF)
OPTION(ENABLE_ALL_WARNINGS "Enable all possible compiler warnings." OFF)

# C/C++ basic setup
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)
SET(CMAKE_C_EXTENSIONS OFF)
SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

# Prevent symbols from leaking
SET(CMAKE_C_VISIBILITY_PRESET hidden)
SET(CMAKE_CXX_VISIBILITY_PRESET hidden)
SET(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Compiler cache support
FIND_PROGRAM(CCACHE_FOUND ccache)
IF(CCACHE_FOUND)
	MESSAGE(STATUS "Compiler cache enabled: ${CCACHE_FOUND}")
	SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ${CCACHE_FOUND})
ENDIF()

# Set compiler info
SET(CC_CLANG FALSE)
SET(CC_GCC FALSE)
SET(CC_ICC FALSE)
SET(CC_MSVC FALSE)

IF("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
	SET(CC_CLANG TRUE)
ELSEIF("${CMAKE_C_COMPILER_ID}" STREQUAL "AppleClang")
	SET(CC_CLANG TRUE)
ELSEIF("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
	SET(CC_GCC TRUE)
ELSEIF("${CMAKE_C_COMPILER_ID}" STREQUAL "Intel")
	SET(CC_ICC TRUE)
ELSEIF("${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC")
	SET(CC_MSVC TRUE)
ENDIF()

# Support LLVM's libc++
IF(CC_CLANG AND USE_CLANG_LIBCPP)
	MESSAGE(STATUS "Using Clang's libc++")
	SET(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}")
	SET(CMAKE_EXE_LINKER_FLAGS "-stdlib=libc++ ${CMAKE_EXE_LINKER_FLAGS}")
	SET(CMAKE_SHARED_LINKER_FLAGS "-stdlib=libc++ ${CMAKE_SHARED_LINKER_FLAGS}")

	# Popular fmt library should be header-only for compatibility in this case.
	ADD_DEFINITIONS(-DFMT_HEADER_ONLY=1)
ENDIF()

# Set operating system info
SET(OS_UNIX FALSE)
SET(OS_LINUX FALSE)
SET(OS_MACOS FALSE)
SET(OS_WINDOWS FALSE)

IF(UNIX)
	SET(OS_UNIX TRUE)
	ADD_DEFINITIONS(-DOS_UNIX=1)
ENDIF()

IF(UNIX AND "${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
	SET(OS_LINUX TRUE)
	ADD_DEFINITIONS(-DOS_LINUX=1)
ENDIF()

IF(UNIX AND APPLE AND "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
	SET(OS_MACOS TRUE)
	ADD_DEFINITIONS(-DOS_MACOS=1)
ENDIF()

IF(WIN32 AND "${CMAKE_SYSTEM_NAME}" MATCHES "Windows")
	SET(OS_WINDOWS TRUE)
	ADD_DEFINITIONS(-DOS_WINDOWS=1)
ENDIF()

# Test if we are on a big-endian architecture
INCLUDE(TestBigEndian)
TEST_BIG_ENDIAN(SYSTEM_BIGENDIAN)

# Check size of various types
INCLUDE(CheckTypeSize)
CHECK_TYPE_SIZE("size_t" SIZEOF_SIZE_T)
CHECK_TYPE_SIZE("void *" SIZEOF_VOID_PTR)

IF(NOT
   "${SIZEOF_VOID_PTR}"
   STREQUAL
   "${SIZEOF_SIZE_T}")
	MESSAGE(FATAL_ERROR "Size of void * and size_t must be the same!")
ENDIF()

# Check threads support (almost nobody implements C11 threads yet! C++11 is fine)
SET(CMAKE_THREAD_PREFER_PTHREAD TRUE)
FIND_PACKAGE(Threads REQUIRED)
SET(HAVE_PTHREADS FALSE)
SET(HAVE_WIN32_THREADS FALSE)

IF(DEFINED "CMAKE_USE_PTHREADS_INIT")
	IF(${CMAKE_USE_PTHREADS_INIT})
		SET(HAVE_PTHREADS TRUE)
		ADD_DEFINITIONS(-DHAVE_PTHREADS=1)
	ENDIF()
ENDIF()

IF(DEFINED "CMAKE_USE_WIN32_THREADS_INIT")
	IF(${CMAKE_USE_WIN32_THREADS_INIT})
		SET(HAVE_WIN32_THREADS TRUE)
		ADD_DEFINITIONS(-DHAVE_WIN32_THREADS=1)
	ENDIF()
ENDIF()

SET(SYSTEM_THREAD_LIBS ${CMAKE_THREAD_LIBS_INIT})

# Add system defines for header features
IF(OS_UNIX)
	# Support for large files (>2GB) on 32-bit systems
	ADD_DEFINITIONS(-D_FILE_OFFSET_BITS=64)

	# POSIX system (Unix, Linux, MacOS X)
	ADD_DEFINITIONS(-D_XOPEN_SOURCE=700)
	ADD_DEFINITIONS(-D_DEFAULT_SOURCE=1)

	IF(OS_MACOS)
		# Use POSIX, not DARWIN_C_SOURCE, as that enables too many extensions and causes collisions in function names,
		# for example swab().
		ADD_DEFINITIONS(-D_POSIX_C_SOURCE=200809)
	ENDIF()
ENDIF()

IF(OS_WINDOWS AND (CC_GCC OR CC_CLANG))
	ADD_DEFINITIONS(-D__USE_MINGW_ANSI_STDIO=1)
ENDIF()

# Enable C/C++ compiler warnings
IF(ENABLE_ALL_WARNINGS AND (CC_GCC OR CC_CLANG))
	# Enable all warnings for GCC / Clang
	SET(WARN_COMMON_FLAGS "-pedantic -Wall -Wextra")
	SET(WARN_C_FLAGS "")
	SET(WARN_CXX_FLAGS "")

	IF(CC_GCC)
		# Enable all useful warnings in GCC one-by-one.
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wunused -Wundef -Wformat=2 -Wuninitialized -Winit-self")
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings")
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wredundant-decls -Wmissing-declarations -Wdouble-promotion")
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wshadow -Wconversion -Wstrict-overflow=5 -Wlogical-op")

		SET(WARN_C_FLAGS "${WARN_C_FLAGS} -Wstrict-prototypes -Wmissing-prototypes -Wnested-externs")
		SET(WARN_C_FLAGS "${WARN_C_FLAGS} -Wbad-function-cast -Wjump-misses-init")

		SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wold-style-cast -Wzero-as-null-pointer-constant")

		IF(CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 6)
			SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wduplicated-cond")
		ENDIF()

		IF(CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 7)
			SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wduplicated-branches -Wrestrict -Wregister")
		ENDIF()

		IF(CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 8)
			SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wmultistatement-macros")
			SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wcatch-value=3 -Wextra-semi")
		ENDIF()

		IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10)
			SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wmismatched-tags -Wredundant-tags")
		ENDIF()

		IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 11)
			SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wenum-conversion")
		ENDIF()

		# This is enabled by -Wall but not useful across multiple compilers.
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wno-unknown-pragmas")
	ENDIF()

	IF(CC_CLANG)
		# Enable all warnings in Clang, then turn off useless ones.
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Weverything -Wno-packed -Wno-padded -Wno-unreachable-code-break")
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wno-disabled-macro-expansion -Wno-reserved-id-macro -Wno-vla")
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wno-covered-switch-default -Wno-float-equal -Wno-cast-align")
		SET(WARN_COMMON_FLAGS "${WARN_COMMON_FLAGS} -Wno-unknown-warning-option")

		SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wno-c++98-compat -Wno-c++98-compat-pedantic")
		SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wno-global-constructors -Wno-exit-time-destructors")

		SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wno-weak-vtables -Wno-date-time -Wno-ctad-maybe-unsupported")
		SET(WARN_CXX_FLAGS "${WARN_CXX_FLAGS} -Wno-return-std-move-in-c++11")
	ENDIF()

	# Apply all flags.
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${WARN_COMMON_FLAGS} ${WARN_C_FLAGS}")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARN_COMMON_FLAGS} ${WARN_CXX_FLAGS}")
ENDIF()

# RPATH settings
SET(CMAKE_MACOSX_RPATH TRUE)
SET(CMAKE_SKIP_BUILD_RPATH FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Linker settings to only link to needed symbols
IF(OS_UNIX AND NOT OS_MACOS)
	# Add --as-needed to linker flags.
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
	SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")
ENDIF()

# Print info summary for debug purposes
MESSAGE(STATUS "Project name is: ${PROJECT_NAME}")
MESSAGE(STATUS "Project version is: ${PROJECT_VERSION}")
MESSAGE(STATUS "Build type is: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Compiler is Clang: ${CC_CLANG}")
MESSAGE(STATUS "Compiler is GCC: ${CC_GCC}")
MESSAGE(STATUS "Compiler is IntelCC: ${CC_ICC}")
MESSAGE(STATUS "Compiler is MS VisualC: ${CC_MSVC}")
MESSAGE(STATUS "OS is Unix: ${OS_UNIX}")
MESSAGE(STATUS "OS is Linux: ${OS_LINUX}")
MESSAGE(STATUS "OS is MacOS: ${OS_MACOS}")
MESSAGE(STATUS "OS is Windows: ${OS_WINDOWS}")
MESSAGE(STATUS "System is big-endian: ${SYSTEM_BIGENDIAN}")
MESSAGE(STATUS "Thread support is PThreads: ${HAVE_PTHREADS}")
MESSAGE(STATUS "Thread support is Win32 Threads: ${HAVE_WIN32_THREADS}")
MESSAGE(STATUS "Thread support libraries: ${SYSTEM_THREAD_LIBS}")
MESSAGE(STATUS "Current C flags are: ${CMAKE_C_FLAGS}")
MESSAGE(STATUS "Current CXX flags are: ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "Final install bindir is: ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}")
MESSAGE(STATUS "Final install libdir is: ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
MESSAGE(STATUS "Final install includedir is: ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}")
