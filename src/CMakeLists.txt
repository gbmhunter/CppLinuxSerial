
option(SERIAL_BUILD_SHARED_LIBS "Build CppLinuxSerial shared library" OFF)

if (SERIAL_BUILD_SHARED_LIBS)
  add_library(CppLinuxSerial SHARED)
else()
  add_library(CppLinuxSerial STATIC)
endif()

add_library(CppLinuxSerial::CppLinuxSerial ALIAS CppLinuxSerial)

target_compile_features(CppLinuxSerial PRIVATE cxx_std_14)

target_sources(CppLinuxSerial
  PRIVATE 
    SerialPort.cpp)

target_include_directories(
  CppLinuxSerial 
    PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
)

## Installing
include(CMakePackageConfigHelpers)

install(
  TARGETS CppLinuxSerial 
  EXPORT CppLinuxSerialTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  EXPORT CppLinuxSerialTargets
  FILE CppLinuxSerialTargets.cmake
  NAMESPACE CppLinuxSerial::
  DESTINATION lib/cmake/CppLinuxSerial
)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/CppLinuxSerial 
  DESTINATION include
)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/CppLinuxSerialConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CppLinuxSerial
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    CppLinuxSerialConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
  FILES 
    "${CMAKE_CURRENT_BINARY_DIR}/CppLinuxSerialConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CppLinuxSerialConfigVersion.cmake"
  DESTINATION lib/cmake/CppLinuxSerial
)
