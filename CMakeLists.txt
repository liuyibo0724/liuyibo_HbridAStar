cmake_minimum_required(VERSION 3.8)
project(LiuYibo_hybridAStar)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(
        includes
        ${OpenCV_INCLUDE_DIRS}
)

add_library(
        planner
        # src/planner.cpp
        src/node.cpp
        src/hybridAStar.cpp
        src/CollisionDetection.cpp
        src/ReedsSheppPath.cpp
        src/smooth.cpp
        src/dynamicVoronoi.cpp
        src/BSpline.cpp
        src/filter.cpp
)

add_executable(testHybridAstar mains/hybridAStar_main.cpp)
target_link_libraries(testHybridAstar planner ${OpenCV_LIBS})

# add_executable(test_a_star test/test_A_star.cpp)
# target_link_libraries(test_a_star planner ${OpenCV_LIBS})

# add_executable(test_map test/test_map.cpp)
# target_link_libraries(test_map planner ${OpenCV_LIBS})

# add_executable(test_RS test/test_ReedsSheppPath.cpp)
# target_link_libraries(test_RS planner ${OpenCV_LIBS})

# add_executable(test_hybrid_aStar test/test_hybrid_aStar.cpp)
# target_link_libraries(test_hybrid_aStar planner ${OpenCV_LIBS})

# add_executable(test_smooth test/test_smooth.cpp)
# target_link_libraries(test_smooth planner ${OpenCV_LIBS})

# add_executable(test_voronoi test/test_dynamicVoronoi.cpp)
# target_link_libraries(test_voronoi planner ${OpenCV_LIBS})