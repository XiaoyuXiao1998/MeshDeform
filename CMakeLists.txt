cmake_minimum_required(VERSION 3.20)
# set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/current/scripts/buildsystems/vcpkg.cmake" CACHE PATH "" FORCE)

set(CMAKE_CXX_STANDARD 23)

project(deformation)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(libigl CONFIG REQUIRED)
find_package(tsl-ordered-map CONFIG REQUIRED)

file(GLOB_RECURSE SRCFILES
    "src/deform/*.cpp"
)
add_library(libmain ${SRCFILES})
target_include_directories(libmain PUBLIC include)
target_precompile_headers(libmain
    PUBLIC "include/pch.hpp"
)
target_link_libraries(libmain
    PUBLIC tsl::ordered_map
    PUBLIC igl::core
    PUBLIC igl::common
    PUBLIC igl::opengl
    PUBLIC igl::opengl_glfw
    PUBLIC imgui::imgui
    PUBLIC Boost::headers
)

add_executable(main "demo/main.cpp")
target_link_libraries(main libmain OpenMP::OpenMP_CXX)