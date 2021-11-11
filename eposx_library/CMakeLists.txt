cmake_minimum_required(VERSION 2.8.3)
project(eposx_library)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ftd2xx EposCmd
)

###########
## Build ##
###########

include_directories(include)

# Processor classification is based on install.sh of the epos command library
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86_64/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86_64/libEposCmd.so.6.5.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i386"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i486"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i586"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i686")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86/libEposCmd.so.6.5.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv6")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v6/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v6/libEposCmd.so.6.5.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v7/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v7/libEposCmd.so.6.5.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v8/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v8/libEposCmd.so.6.5.1.0)
else()
  message(FATAL_ERROR "Unsupported processor type: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

add_library(ftd2xx SHARED ${ftd2xx_LIBRARY})
add_custom_command(TARGET ftd2xx POST_BUILD COMMAND cp ${ftd2xx_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libftd2xx.so)
set_target_properties(ftd2xx PROPERTIES LINKER_LANGUAGE CXX)

add_library(EposCmd SHARED ${EposCmd_LIBRARY})
add_custom_command(TARGET EposCmd POST_BUILD COMMAND cp ${EposCmd_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libEposCmd.so)
set_target_properties(EposCmd PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

install(TARGETS ftd2xx EposCmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
