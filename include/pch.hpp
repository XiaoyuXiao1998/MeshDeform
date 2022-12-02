#pragma once

#include <imgui.h>
namespace ImGui {
    static inline float GetContentRegionAvailWidth() { return GetContentRegionAvail().x; } // a quick fix for the OBSOLETED function
};

#include <tsl/ordered_map.h>
#include <boost/any.hpp>
#include <Eigen/Core>

#include <igl/unproject_onto_mesh.h>
#include <igl/opengl/ViewerData.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
