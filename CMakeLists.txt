cmake_minimum_required(VERSION 3.1.0)
project(CppLinuxSerial)

add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "If set to true, unit tests will be build as part of make all." TRUE)
if (BUILD_TESTS)
    message("BUILD_TESTS=TRUE, unit tests will be built.")
else ()
    message("BUILD_TESTS=FALSE, unit tests will NOT be built.")
endif ()

#=================================================================================================#
#========================================= gtest INSTALL =========================================#
#=================================================================================================#

if (BUILD_TESTS)
    # Download and unpack googletest at configure time
    configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
            RESULT_VARIABLE result
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
    if(result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
            RESULT_VARIABLE result
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
    if(result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
            ${CMAKE_BINARY_DIR}/googletest-build
            EXCLUDE_FROM_ALL)

    # The gtest/gtest_main targets carry header search path
    # dependencies automatically when using CMake 2.8.11 or
    # later. Otherwise we have to add them here ourselves.
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
        include_directories("${gtest_SOURCE_DIR}/include")
    endif()

endif()

#=================================================================================================#
#========================================= This Project ==========================================#
#=================================================================================================#

# Now simply link your own targets against gtest, gmock,
# etc. as appropriate
include_directories(include)

add_subdirectory(src)
if(BUILD_TESTS)
    add_subdirectory(test/unit)
endif()
