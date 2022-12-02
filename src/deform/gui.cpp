#include "deform/gui.hpp"

#include <format>

bool MeshDeformerGui::DeformerControllerPlugin::
mouse_up(int button, int modifier) {
    ImGui_ImplGlfw_MouseButtonCallback(this->viewer->window, button, GLFW_RELEASE, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool MeshDeformerGui::DeformerControllerPlugin::
post_load() {
    if (this->callback_post_load) { callback_post_load(); }
    return false;
}

bool MeshDeformerGui::DeformerControllerPlugin::
load(std::string filename) {
    if (this->callback_pre_load) { callback_pre_load(); }
    return false;
}

MeshDeformerGui::MeshDeformerGui() {
    viewer.plugins.push_back(&this->controller);
    viewer.core().orthographic = true;
}

MeshDeformerGui::~MeshDeformerGui() {

}

void MeshDeformerGui::
add_deformer(std::string name, AbstractMeshDeformer& deformer) {
    this->deformers_names.push_back(name);
    this->deformers.emplace_back(&deformer);
}

void MeshDeformerGui::
add_params(std::string name, AnyParams& params) {
    this->params_names.push_back(name);
    this->params.emplace_back(&params);
}

void MeshDeformerGui::
launch(std::string filename) {
    this->init_imgui_window();

    this->init_load_mesh_callbacks();
    this->init_handle_editor_callbacks();

    this->bind_deformers_data();

    if (filename.size() > 0) {
        this->viewer.load_mesh_from_file(filename);
    }
    this->viewer.launch(true, false, "Mesh Deformer", window_width, window_height);
}

void MeshDeformerGui::
init_load_mesh_callbacks() {
    this->controller.callback_pre_load = [this]() {
        // Clear all mesh before loading new one
        viewer.selected_data_index = viewer.data_list.size()-1;
        while(viewer.erase_mesh(viewer.selected_data_index)){};
        viewer.data().clear();
    };

    this->controller.callback_post_load = [this]() {
        // Trigger the deformer new mesh is coming
        auto& data = this->viewer.data();
        this->V_origin = data.V.cast<float>();
        for (auto* deformer : this->deformers) {
            deformer->on_load_mesh(
                this->V_origin,
                data.F,
                data
            );
        }
    };
}

void MeshDeformerGui::
lock_rest_handles_callbacks() {
    for (auto* deformer : this->deformers) {
        deformer->on_lock_rest_handles();
    }
}

void MeshDeformerGui::
update_handles_viz() {
    if (this->is_editing_handles) {
        Eigen::RowVector3d color;
        Eigen::RowVector3d selected_color;
        Eigen::MatrixX3f handles_mat;

        if (this->is_placing_handles) {
            color = Eigen::RowVector3d(0.0, 0.8, 0.0);
            selected_color = Eigen::RowVector3d(0.0, 1.0, 0.0);
            handles_mat = this->handles.get_handles_rest();
        } else {
            color = Eigen::RowVector3d(0.8, 0.8, 0.8);
            selected_color = Eigen::RowVector3d(0.5, 0.5, 0.5);
            handles_mat = this->handles.get_handles();
        }
        this->viewer.data().set_points(handles_mat.cast<double>(), color);
        if (this->selected_handle_idx != -1) {
            this->viewer.data().set_points(handles_mat.row(this->selected_handle_idx).cast<double>(), selected_color);
        }
    } else {
        this->viewer.data().clear_points();
    }
}

void MeshDeformerGui::
update_mesh_deform() {
    if (!this->is_placing_handles) {
        auto& cur_deformer = this->deformers[this->deformer_choice];
        cur_deformer->deform_mesh(this->V_deformed);
        this->viewer.data().set_vertices(this->V_deformed.cast<double>());
    } else {
        this->viewer.data().set_vertices(this->V_origin.cast<double>());
    }
}

void MeshDeformerGui::
init_handle_editor_callbacks() {
    this->viewer.callback_mouse_down = [this](
        igl::opengl::glfw::Viewer &viewer,
        int button,
        int mod
    ) {
        this->last_mouse = Eigen::RowVector3f(
            static_cast<float>(this->viewer.current_mouse_x),
            this->viewer.core().viewport(3) - viewer.current_mouse_y,
            0
        );
        Eigen::Matrix<float, -1, 3, 1>& handles = this->is_placing_handles ? this->handles.get_handles_rest() : this->handles.get_handles();
        size_t handle_count = this->is_placing_handles ? this->handles.get_handles_rest_count() : this->handles.get_handles_count();

        if (this->is_editing_handles) {
            if (handle_count > 0) {
                Eigen::MatrixXf handles_screen_space;
                igl::project(
                    handles,
                    this->viewer.core().view,
                    this->viewer.core().proj,
                    this->viewer.core().viewport,
                    handles_screen_space
                );
                Eigen::RowVectorXf distances = (
                    handles_screen_space.rowwise() - this->last_mouse
                ).rowwise().norm();

                if (distances.minCoeff(&this->selected_handle_idx) > 30) {
                    this->selected_handle_idx = -1;
                    if (this->is_placing_handles) {
                        this->place_handle();
                    }
                }
            } else {
                if (this->is_placing_handles) {
                    this->place_handle();
                }
            }
        }
        this->update_handles_viz();
        return this->is_editing_handles && this->selected_handle_idx != -1;
    };

    this->viewer.callback_mouse_move = [this](
        igl::opengl::glfw::Viewer &viewer,
        int mouse_x,
        int mouse_y
    ) {
        if (this->is_editing_handles) {
            if (this->selected_handle_idx != -1) {
                Eigen::RowVector3f drag_mouse(
                    static_cast<float>(viewer.current_mouse_x),
                    viewer.core().viewport(3) - viewer.current_mouse_y,
                    last_mouse(2)
                );
                Eigen::RowVector3f drag_scene, last_scene;
                igl::unproject(
                    drag_mouse,
                    viewer.core().view,
                    viewer.core().proj,
                    viewer.core().viewport,
                    drag_scene
                );
                igl::unproject(
                    last_mouse,
                    viewer.core().view,
                    viewer.core().proj,
                    viewer.core().viewport,
                    last_scene
                );
                if (this->is_placing_handles) {
                    this->handles.update_handle_rest(this->selected_handle_idx,
                        this->handles.get_handle_rest(this->selected_handle_idx) +
                        (drag_scene - last_scene)
                    );
                } else {
                    this->handles.update_handle(this->selected_handle_idx,
                        this->handles.get_handle(this->selected_handle_idx) +
                        (drag_scene - last_scene)
                    );
                }
                this->last_mouse = drag_mouse;
                this->update_mesh_deform();
            }
        }
        this->update_handles_viz();
        return this->is_editing_handles && this->selected_handle_idx != -1;
    };

    this->viewer.callback_mouse_up = [this](
        igl::opengl::glfw::Viewer &viewer,
        int button,
        int modifier
    ) {
        this->selected_handle_idx = -1;
        this->update_handles_viz();
        return false;
    };
}

void MeshDeformerGui::
on_params_choice_updated() {
    for (auto* deformer : this->deformers) {
        deformer->bind_data(this->params[this->params_choice]);
    }
    this->update_mesh_deform();
}

void MeshDeformerGui::
on_deformer_choice_updated() {
    this->update_mesh_deform();
}

void MeshDeformerGui::
bind_deformers_data() {
    for (auto* deformer : this->deformers) {
        deformer->bind_data(this->params[this->params_choice], &this->handles);
    }
}

void MeshDeformerGui::
place_handle() {
    int fid;
    Eigen::Vector3f bary;
    const auto& F = this->viewer.data().F;

    if (igl::unproject_onto_mesh(
        this->last_mouse.head(2),
        this->viewer.core().view,
        this->viewer.core().proj,
        this->viewer.core().viewport,
        this->V_origin, F,
        fid, bary)
    ) {
        long c;
        bary.maxCoeff(&c);
        Eigen::RowVector3f projected_V = this->V_origin.row(F(fid,c));
        this->handles.add_handle(projected_V);
    }
}

void MeshDeformerGui::
init_imgui_window() {
    this->controller.callback_draw_custom_window = [this]() {
        // Define next window position + size
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(window_width), 0), ImGuiCond_FirstUseEver, ImVec2(1, 0));
        ImGui::SetNextWindowSize(ImVec2(260, 160), ImGuiCond_FirstUseEver);

        ImGui::Begin(
            "Deformer Controller",
            &this->is_controller_visible,
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize
        );

        const char * switch_button_text1;
        if (this->is_editing_handles) { switch_button_text1 = "Stop Editing Handles"; }
        else { switch_button_text1 = "Start Editing Handles"; }

        if (ImGui::Button(switch_button_text1, ImVec2(-1, 0))) {
            this->is_editing_handles = !this->is_editing_handles;
            this->update_handles_viz();
        }

        if (this->is_editing_handles) {
            const char * switch_button_text2;
            if (this->is_placing_handles) { switch_button_text2 = "Finish Editing Rest Handles"; }
            else { switch_button_text2 = "Switch to Edit Rest Handles"; }

            if (ImGui::Button(switch_button_text2, ImVec2(-1, 0))) {
                if (this->is_placing_handles) {
                    this->handles.lock_rest_handles();
                    this->lock_rest_handles_callbacks();
                }
                this->is_placing_handles = !this->is_placing_handles;
                this->update_handles_viz();
                this->update_mesh_deform();
            }

            if (this->is_placing_handles) {
                if (ImGui::Button("Clear Handles", ImVec2(-1, 0))) {
                    this->handles.clear_handles();
                    this->update_handles_viz();
                }
            } else {
                if (ImGui::Button("Reset Handles", ImVec2(-1, 0))) {
                    this->handles.reset_handles();
                    this->update_handles_viz();
                    this->update_mesh_deform();
                }
            }
        }

        if (ImGui::CollapsingHeader("Handles Postions", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (int i = 0; i < this->handles.get_handles_count(); ++i) {
                std::string name = std::format("handle_{}", i);
                if (ImGui::DragFloat3(name.c_str(), this->handles.get_handle_ptr(i))) {
                    this->update_handles_viz();
                    this->update_mesh_deform();
                }
            }
        }
        if (ImGui::CollapsingHeader("Handles At Rest", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (int i = 0; i < this->handles.get_handles_rest_count(); ++i) {
                std::string name = std::format("handle_{}", i);
                if (ImGui::DragFloat3(name.c_str(), this->handles.get_handle_rest_ptr(i))) {
                    this->lock_rest_handles_callbacks();
                    this->update_handles_viz();
                    this->update_mesh_deform();
                };
            }
        }

        if (ImGui::CollapsingHeader("Mesh Deformers", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (int i = 0; i < this->deformers_names.size(); ++i) {
                if (ImGui::RadioButton(this->deformers_names[i].c_str(), &this->deformer_choice, i)) {
                    this->on_deformer_choice_updated();
                }
            }
        }

        if (ImGui::CollapsingHeader("Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
            for (int i = 0; i < this->params_names.size(); ++i) {
                if (ImGui::RadioButton(this->params_names[i].c_str(), &this->params_choice, i)) {
                    this->on_params_choice_updated();
                }
            }
            if (this->params_choice >= 0) {
                imgui_params_editor(*this->params[this->params_choice]);
            }
        }

        // Add a button
        if (ImGui::Button("Test", ImVec2(-1,0))) {
            std::cout << "Test Button Clicked!" << std::endl;
        }

        ImGui::End();
    };
}

void MeshDeformerGui::
imgui_params_editor(AnyParams& params) {
    for (auto& iter : params) {
        const std::string& name = iter.first;
        auto& type = iter.second.type();
        boost::any* anyval_ptr = const_cast<boost::any *>(&iter.second);

        if (type == typeid(float)) {
            float* valptr = boost::any_cast<float>(anyval_ptr);
            ImGui::DragFloat(name.c_str(), valptr);
        } else if (type == typeid(double)) {
            double* valptr = boost::any_cast<double>(anyval_ptr);
            ImGui::InputDouble(name.c_str(), valptr);
        } else if (type == typeid(int)) {
            int* valptr = boost::any_cast<int>(anyval_ptr);
            ImGui::DragInt(name.c_str(), valptr);
        } else if (type == typeid(bool)) {
            bool* valptr = boost::any_cast<bool>(anyval_ptr);
            ImGui::Checkbox(name.c_str(), valptr);
        } else if (type == typeid(Eigen::Vector2f)) {
            Eigen::Vector2f* valptr = boost::any_cast<Eigen::Vector2f>(anyval_ptr);
            ImGui::DragFloat2(name.c_str(), valptr->data());
        } else if (type == typeid(Eigen::Vector3f)) {
            Eigen::Vector3f* valptr = boost::any_cast<Eigen::Vector3f>(anyval_ptr);
            ImGui::DragFloat3(name.c_str(), valptr->data());
        } else if (type == typeid(std::string)) {
            ImGui::InputText(name.c_str(), *boost::any_cast<std::string>(anyval_ptr));
        }
    }
}