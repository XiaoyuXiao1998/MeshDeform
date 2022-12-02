#pragma once

#include "deform/base.hpp"

class ARAPMeshDeformer : public AbstractMeshDeformer {
public:
    ARAPMeshDeformer(std::string name = "default") : name(name) {};
    void init_params(AnyParams& params) override;
    void on_load_mesh(
        const Eigen::MatrixX3f& V_in,
        const Eigen::MatrixX3i& F_in,
        const igl::opengl::ViewerData& extras
    ) override;
    void on_lock_rest_handles() override;
    void deform_mesh(Eigen::MatrixX3f& V_out) override;

private:

    //initialization functions and variables
    Eigen::MatrixXf weightMatrix;
    void initializeWeightMatrix();

    // Neighborhood of vertices (Mapping between vertex id and its neighbor ids)
    std::map<int, std::vector<int>> neighborhood;
    void initializeNeighborhood();

    void computeLMatrix();
    // Laplace matrix
    Eigen::MatrixXf LMatrix;

    //handle index;
    std::vector<int> handleIndex;



private:
    const Eigen::MatrixX3f* V_in;
    const Eigen::MatrixX3i* F_in;
    std::string name;

    // Eigen solver
    Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
    //local step: estimate rotation matrix locally
    std::vector<Eigen::Matrix3f>  localStep(Eigen::MatrixX3f & deformedVertices);
    Eigen::MatrixXf compute_b( std::vector<Eigen::Matrix3f>  &);
    void globalStep(Eigen::MatrixX3f &deformedVertices,std::vector<Eigen::Matrix3f>  &rotationMatrices);
    float computeEnergy(Eigen::MatrixX3f &deformedVertices,std::vector<Eigen::Matrix3f>  &rotationMatrices);


};