#pragma once

#include "pch.hpp"

#include <vector>
#include <functional>

#include "deform/base.hpp"

class MeshDeformerGui final {
private:
    class DeformerControllerPlugin : public igl::opengl::glfw::imgui::ImGuiMenu {
    public:
        bool mouse_up(int button, int modifier) override; // fix the mouse_up event stolen bug
        bool post_load() override;
        bool load(std::string) override; // override the load plugin function for a preload operation

        std::function<void(void)> callback_post_load;
        std::function<void(void)> callback_pre_load;
    };

public:
    MeshDeformerGui();
    ~MeshDeformerGui();

    igl::opengl::glfw::Viewer viewer;
    MeshDeformerGui::DeformerControllerPlugin controller;

    void add_deformer(std::string name, AbstractMeshDeformer& deformer);
    void add_params(std::string name, AnyParams& params);
    void launch(std::string filename = "");

    const int window_width = 1200;
    const int window_height = 720;

private:
    void init_imgui_window();

    void init_load_mesh_callbacks();
    void update_handles_viz();
    void update_mesh_deform();
    void init_handle_editor_callbacks();

    void on_deformer_choice_updated();
    void on_params_choice_updated();
    void lock_rest_handles_callbacks();
    void bind_deformers_data();
    void place_handle();

    AbstractMeshDeformer* get_current_deformer() { return this->deformers[this->deformer_choice]; }
    AnyParams* get_current_params() { return this->params[this->params_choice]; }

    void imgui_params_editor(AnyParams& params);

    HandleManager handles;

    bool is_controller_visible = true;
    bool auto_deform = true;
    bool is_editing_handles = false;
    bool is_placing_handles = true;

    std::vector<AbstractMeshDeformer*> deformers;
    std::vector<AnyParams*> params;
    std::vector<std::string> deformers_names;
    std::vector<std::string> params_names;

    int deformer_choice = 0;
    int params_choice = 0;
    Eigen::RowVector3f last_mouse;

    Eigen::MatrixX3f V_origin;
    Eigen::MatrixX3f V_deformed;
    int selected_handle_idx = -1;
};