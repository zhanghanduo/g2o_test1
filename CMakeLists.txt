cmake_minimum_required( VERSION 2.8 )
project( g2o_ba_example )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -Wall -O2 -march=native" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.2 REQUIRED)
find_package(Cholmod)
#find_package( CSparse REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

# add_executable(ba_example main.cpp )
add_executable(test_node test.cpp)
add_executable(orb_node orb.cpp orb_edge.cpp orb_edge.h)
add_executable(orb_se3_node orb_se3.cpp orb_edge.cpp orb_edge.h)
# target_link_libraries( ba_example 
    # ${OpenCV_LIBS}
    # g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
    # ${CHOLMOD_LIBRARIES}
    # )
target_link_libraries(test_node
        ${OpenCV_LIBS}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
        ${CHOLMOD_LIBRARIES}
        csparse
        ${EIGEN3_LIBRARIES}
        g2o_csparse_extension
        )
target_link_libraries(orb_node
        ${OpenCV_LIBS}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba g2o_solver_dense
        ${CHOLMOD_LIBRARIES}
        csparse
        ${EIGEN3_LIBRARIES}
        g2o_csparse_extension
        )
target_link_libraries(orb_se3_node
        ${OpenCV_LIBS}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba g2o_solver_dense
        ${CHOLMOD_LIBRARIES}
        csparse
        ${EIGEN3_LIBRARIES}
        g2o_csparse_extension
        )