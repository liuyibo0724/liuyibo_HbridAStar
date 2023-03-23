cmake_minimum_required(VERSION 2.8.12)

project(sysd_framework LANGUAGES C CXX)

# -------------------------------------------------------------------------------
# Setting up dirctory variables
# -------------------------------------------------------------------------------
set(IDC_PROJECT_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
# -------------------------------------------------------------------------------
#Config compile options
# -------------------------------------------------------------------------------
#add_compile_options(-D_DEBUG)           #for turn fastdds log info on
#add_compile_options(-D_INTERNALDEBUG)   #for turn fastdds log info on

# -------------------------------------------------------------------------------
#Find requirements
# -------------------------------------------------------------------------------
find_package(fastcdr REQUIRED PATHS ${CMAKE_PREFIX_PATH})
find_package(fastrtps REQUIRED PATHS ${CMAKE_PREFIX_PATH})
find_package(Boost REQUIRED COMPONENTS
    system
    thread
    chrono
    date_time
    random
    iostreams
    # PATHS ${CMAKE_PREFIX_PATH}
)

# -------------------------------------------------------------------------------
# Add include directories
# -------------------------------------------------------------------------------
include_directories(
    ${CMAKE_PREFIX_PATH}/include
    ${IDC_PROJECT_SOURCE_DIR}/
)

# -------------------------------------------------------------------------------
# Add source files
# -------------------------------------------------------------------------------
file(GLOB_RECURSE SYSD_FRAMEWORK_SRC_FILES 
    ${IDC_PROJECT_SOURCE_DIR}/idl_generated/*.cxx
    ${IDC_PROJECT_SOURCE_DIR}/sysd_framework/sysd_scheduler.cpp
)

# -------------------------------------------------------------------------------
# link idds libraries
# -------------------------------------------------------------------------------
add_library(${PROJECT_NAME} ${SYSD_FRAMEWORK_SRC_FILES})
link_libraries(
    ${PROJECT_NAME}
    fastcdr
    fastrtps
)

SET(CMAKE_INSTALL_PREFIX "${PARTLIB}")
INSTALL(DIRECTORY . DESTINATION ${CMAKE_INSTALL_PREFIX}/include/sysd_framework  FILES_MATCHING PATTERN "*.h*")
INSTALL(DIRECTORY . DESTINATION ${CMAKE_INSTALL_PREFIX}/include/sysd_framework FILES_MATCHING PATTERN "*.idl")
