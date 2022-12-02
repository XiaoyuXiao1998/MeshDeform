#include "deform/example.hpp"

#include <string>
#include <iostream>
#include <format>

using namespace Eigen;

void ExampleMeshDeformer::
init_params(AnyParams& params) {
    params.init<bool>("flag", true);
    params.init<int>("index", -1);
    params.init<float>("alpha", 0.0f);
    params.init<Eigen::Vector2f>("pos2d", Eigen::Vector2f::Zero());
    params.init<Eigen::Vector3f>("pos3d", Eigen::Vector3f::Zero());
    params.init<std::string>("name", "1");
}

void ExampleMeshDeformer::
on_load_mesh(
    const Eigen::MatrixX3f& V_in,
    const Eigen::MatrixX3i& F_in,
    const igl::opengl::ViewerData& extras
) {
    std::cout << std::format("Deformer {} on_load_mesh:", this->name) << std::endl;
    std::cout << "Mesh is loaded!" << std::endl;
    std::cout << "Vertice Count: " << V_in.rows() << std::endl;
    std::cout << "Face Count: " << F_in.rows() << std::endl;
    this->V_in = &V_in;
}

void ExampleMeshDeformer::
on_lock_rest_handles() {
    std::cout << std::format("Deformer {} on_lock_rest_handles:", this->name) << std::endl;
    std::cout << "Handle Count: " << this->handleManager->get_handles_count() << std::endl;
    std::cout << "Handle Data: " << std::endl;
    for (int i = 0; i < this->handleManager->get_handles_count(); ++i) {
        std::cout << "[" << i << "]"
                  << this->handleManager->get_handle(i) << std::endl;
    }
}

void ExampleMeshDeformer::
deform_mesh(
    Eigen::MatrixX3f& V_out
) {
    std::cout << std::format("Deformer {} on_deform_mesh:", this->name) << std::endl;

    auto ratio = this->handleManager->get_handles().sum() / this->handleManager->get_handles_rest().sum();
    V_out = ratio * (*this->V_in);
}
