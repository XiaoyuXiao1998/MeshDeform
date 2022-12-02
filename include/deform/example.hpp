#pragma once

#include "deform/base.hpp"

class ExampleMeshDeformer : public AbstractMeshDeformer {
public:
    ExampleMeshDeformer(std::string name = "default") : name(name) {};
    void init_params(AnyParams& params) override;
    void on_load_mesh(
        const Eigen::MatrixX3f& V_in,
        const Eigen::MatrixX3i& F_in,
        const igl::opengl::ViewerData& extras
    ) override;
    void on_lock_rest_handles() override;
    void deform_mesh(Eigen::MatrixX3f& V_out) override;

private:
    const Eigen::MatrixX3f* V_in;
    std::string name;
};