cmake_minimum_required(VERSION 3.17)
project(CollisionDetector CUDA)

set(CMAKE_CUDA_STANDARD 14)

include_directories(
        include
)
aux_source_directory(src SOURCE)
add_executable(CollisionDetector ${SOURCE})

set_target_properties(
        CollisionDetector
        PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
)