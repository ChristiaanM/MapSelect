cmake_minimum_required(VERSION 3.10)
project(MapSelect)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(CXX ON) # Used by Gurobi
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=native")  
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

################################################################################
# Options

option(OPTION_GUROBI_FEATURES "Build with GUROBI" ON)
option(OPTION_CHOLMOD_FEATURES "Build with CHOLMOD" ON)
message(STATUS "${OPTION_GUROBI_FEATURES}")
message(STATUS "${OPTION_CHOLMOD_FEATURES}")

################################################################################


###

# FIND PACKAGES

###
message(STATUS "Searching for packages....")


find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED) 
find_package(OpenMP)


if (OPENMP_FOUND)
    message(STATUS "OpenMP found")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
find_package(OpenCV 3.0 REQUIRED)
find_package(Boost 1.65.1 COMPONENTS program_options REQUIRED)
find_package(GUROBI)
include(FindProtobuf)
find_package(Protobuf 3.21.1 REQUIRED)


set(MSELECT_CHOLMOD_SOURCES "${PROJECT_SOURCE_DIR}/mapselect/functions/SLAMInfoCholmod.cpp")
set(MSELECT_GUROBI_SOURCES "${PROJECT_SOURCE_DIR}/mapselect/algorithms/GurobiOpt.cpp")

set(MSELECT_BASE_LIBS Eigen3::Eigen gtsam gtsam_unstable ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS})
set(MSELECT_CHOLMOD_LIBS ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${CHOLMOD_LIBRARIES} suitesparseconfig ) 
set(MSELECT_CHOLMOD_INCLUDE ${BLAS_INCLUDE} ${CHOLMOD_INCLUDES} ${CHOLMOD_INCLUDE_DIR})
set(MSELECT_GUROBI_LIBS ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY} )
set(MSELECT_GUROBI_INCLUDE ${GUROBI_INCLUDE_DIRS})

set(CHOLMOD_FEATURES ${OPTION_CHOLMOD_FEATURES})
set(GUROBI_FEATURES ${OPTION_GUROBI_FEATURES})

if (CHOLMOD_LIBRARIES STREQUAL "")
set(CHOLMOD_FEATURES OFF)
endif()

if (NOT GUROBI_FOUND)
set(GUROBI_FEATURES OFF)
endif()

if (NOT CHOLMOD_FEATURES)
message(STATUS "${CHOLMOD_LIBRARIES}")
set(MSELECT_CHOLMOD_SOURCES "")
set(MSELECT_CHOLMOD_LIBS "")
set(MSELECT_CHOLMOD_INCLUDE "")
message(STATUS "NOT USING CHOLMOD")
endif()

if (NOT GUROBI_FEATURES)
set(MSELECT_GUROBI_SOURCES "")
set(MSELECT_GUROBI_LIBS "")
set(MSELECT_GUROBI_INCLUDE "")
message(STATUS "NOT USING GUROBI")
endif()


add_library(MapSelect SHARED
${PROJECT_SOURCE_DIR}/thirdparty/osmap/osmap.pb.cc
${PROJECT_SOURCE_DIR}/mapselect/maps/OSMapData.cpp
${PROJECT_SOURCE_DIR}/mapselect/utils/BucketQueue.cpp
${PROJECT_SOURCE_DIR}/mapselect/slam/SLAMGraphGTSAM.cpp
${PROJECT_SOURCE_DIR}/mapselect/slam/SLAMGraph.cpp
${PROJECT_SOURCE_DIR}/mapselect/functions/SLAMInfoGTSAM.cpp 
${PROJECT_SOURCE_DIR}/mapselect/functions/SLAMInfoDense.cpp 

${MSELECT_CHOLMOD_SOURCES}
${MSELECT_GUROBI_SOURCES}
)


target_include_directories(MapSelect PUBLIC 
${MSELECT_CHOLMOD_INCLUDE}
${MSELECT_GUROBI_INCLUDE}
${PROTOBUF_INCLUDE_DIR}
)



target_link_libraries(MapSelect PUBLIC
${MSELECT_CHOLMOD_LIBS}
${MSELECT_GUROBI_LIBS}
${MSELECT_BASE_LIBS}
)


target_include_directories(MapSelect PUBLIC

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>  
$<INSTALL_INTERFACE:include/mapselect/>

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/osmap/>
$<INSTALL_INTERFACE:include/mapselect/thirdparty/osmap/>
)

message(STATUS ${MSELECT_BASE_LIBS})

add_executable(mselect 
mapselect/mselect.cpp 
)

if (GUROBI_FEATURES)
target_compile_definitions(mselect  PRIVATE GUROBI_FEATURES=1)
endif()

if (CHOLMOD_FEATURES)
target_compile_definitions(mselect  PRIVATE CHOLMOD_FEATURES=1)
endif()


target_link_libraries(mselect MapSelect Boost::program_options )
target_include_directories(mselect PUBLIC ${Boost_INCLUDE_DIRS})


add_executable(demo
mapselect/demo.cpp 
)

if (GUROBI_FEATURES)
target_compile_definitions(demo  PRIVATE GUROBI_FEATURES=1)
endif()

if (CHOLMOD_FEATURES)
target_compile_definitions(demo  PRIVATE CHOLMOD_FEATURES=1)
endif()

target_link_libraries(demo MapSelect)



add_executable(setcompare mapselect/setcompare.cpp)
target_link_libraries(setcompare MapSelect)

install(TARGETS MapSelect 
EXPORT MapSelect-export 
LIBRARY DESTINATION lib   
ARCHIVE DESTINATION lib
) 

install(EXPORT MapSelect-export
FILE
MapSelectTargets.cmake
NAMESPACE
MapSelect::
DESTINATION
lib/cmake/MapSelect
)


install(DIRECTORY "${CMAKE_SOURCE_DIR}/mapselect/" 
        DESTINATION "include/mapselect/mapselect" 
        FILES_MATCHING 
        PATTERN "*.h"
)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/thirdparty/" 
        DESTINATION "include/mapselect/thirdparty" 
        FILES_MATCHING 
        PATTERN "*.h"
)


install(FILES
${CMAKE_CURRENT_SOURCE_DIR}/cmake/MapSelectConfig.cmake
DESTINATION "lib/cmake/MapSelect" 
)













