cmake_minimum_required( VERSION 2.8 )
project( depth_filter )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O2 -fopenmp -DBOOST_MATH_DISABLE_FLOAT128" )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# BOOST
FIND_PACKAGE( Boost REQUIRED COMPONENTS thread system )
include_directories( ${Boost_INCLUDE_DIRS} )

# PCL
find_package( PCL REQUIRED COMPONENTS common io )
include_directories( ${PCL_INCLUDE_DIRS} )

# ALL header
include_directories( ${PROJECT_SOURCE_DIR} )

# all source
set( ALL_SOURCE
    ${PROJECT_SOURCE_DIR}/utils.cc
    ${PROJECT_SOURCE_DIR}/depth_filter.cc
)

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable( depth_filter main.cc ${ALL_SOURCE} )
target_link_libraries( depth_filter ${THIRD_PARTY_LIBS} )