#include "deform/ARAP.hpp"

#include <string>
#include <iostream>
#include <format>
#include <math.h>
#include <chrono>
#include <thread>

#define M_PI 3.14159265358979323846
#include <omp.h>

using namespace Eigen;

void ARAPMeshDeformer::
init_params(AnyParams& params) {
    params.init<int>("max_iter", 10);
    params.init<float>("threshold", 0.8f);
}

void ARAPMeshDeformer::
on_load_mesh(
    const Eigen::MatrixX3f& V_in,
    const Eigen::MatrixX3i& F_in,
    const igl::opengl::ViewerData& extras
) {
    std::cout << std::format("Deformer {} on_load_mesh:", this->name) << std::endl;
    std::cout << "Mesh is loaded and ARAP deformer is using!" << std::endl;
    std::cout << "Vertice Count: " << V_in.rows() << std::endl;
    std::cout << "Face Count: " << F_in.rows() << std::endl;
    this->V_in = &V_in;
    this->F_in = &F_in;

    //initialization
    std::cout<<" initialize neighborhood of each vertex at ARAP deformer" <<std::endl;
    initializeNeighborhood();

    std::cout<<" precompute weight matrix at ARAP deformer " <<std::endl;
    initializeWeightMatrix();



}

void ARAPMeshDeformer::
on_lock_rest_handles() {
    std::cout << std::format("Deformer {} on_lock_rest_handles:", this->name) << std::endl;
    std::cout << "Handle Count: " << this->handleManager->get_handles_count() << std::endl;
    std::cout << "Handle Data: " << std::endl;
    for (int i = 0; i < this->handleManager->get_handles_count(); ++i) {
        std::cout << "[" << i << "]"
                  << this->handleManager->get_handle(i) << std::endl;
    }

    std::cout<<"assign handle data to neareast index "<< std::endl;
    handleIndex = std::vector<int>(this->handleManager->get_handles_count(),-1);
    std::vector<float> nearestDisatance(this->handleManager->get_handles_count(),std::numeric_limits<float>::max());
    for (int i = 0; i < V_in->rows();i++){
        for (int j = 0; j < this->handleManager->get_handles_count(); j++) {
            float distance = (this->handleManager->get_handle(j) - V_in->row(i)).norm();
            if(distance < nearestDisatance[j]){
                nearestDisatance[j] = distance;
                handleIndex[j] = i;
            }
        }
    }

    std::cout<< " handle index in the vertices"<<std::endl;
    for (int i = 0; i < this->handleManager->get_handles_count(); ++i) {
        std::cout << "[" << i << "]"<<" "
                  << handleIndex[i] <<" "
                  <<nearestDisatance[i]<<std::endl;
    }

    std::cout<<" compute L matrix at ARAP deformer " <<std::endl;
    computeLMatrix();

    std::cout<< "update L matrix for handle points"<<std::endl;
    for (int i = 0; i < this->handleManager->get_handles_count(); ++i) {
        int index = handleIndex[i];
        LMatrix.row(index).setZero();
        LMatrix(index,index) = 1;

    }







}

void ARAPMeshDeformer::
deform_mesh(
    Eigen::MatrixX3f& V_out
) {
    std::cout << std::format("Deformer {} on_deform_mesh:", this->name) << std::endl;

    //Initial guess of positions
    // V_out is deformed vertices
    // Initial guess
    if(LMatrix.cols() == V_out.rows()) {
        solver.compute(LMatrix.sparseView());
        Eigen::MatrixX3f deformedVertices = solver.solve(LMatrix * (*V_in));
        float energy = 100;

        int max_iter = this->params->get<int>("max_iter");
        float threshold = this->params->get<float>("threshold");

        for(int iter = 0; iter < max_iter && energy >= threshold; iter ++  ) {
            std::vector<Eigen::Matrix3f> rotationMatrices = localStep(deformedVertices);
            globalStep(deformedVertices, rotationMatrices);
            energy =  computeEnergy(deformedVertices, rotationMatrices);
            printf("Iteration %i: rigidity energy = %.4f\n", iter + 1, energy );
        }
        V_out = deformedVertices;

    }
    else{
        std::cout<<"initialize Vout "<<std::endl;
        V_out =  (*this->V_in);
    }
}

void ARAPMeshDeformer::
initializeNeighborhood() {
    // Iterate over all faces
    for (int i = 0; i < V_in->rows(); i++) {

        std::vector<int> allNeighbors;
        std::vector<int> uniqueNeighbors;


        // Iterate over the faces
        for (int j = 0; j < F_in->rows(); j++) {
            for (int k = 0; k < 3; k++) {
                if ((*F_in)(j, k) == i) {
                    allNeighbors.push_back((*F_in)(j, (k + 1) % 3));
                    allNeighbors.push_back((*F_in)(j, (k + 2) % 3));
                }
            }
        }

        for (auto neighbor: allNeighbors) { // Iterate over all the found neighbors of the vertex
            // Push to distinct neighbors if it is not in the list
            if (std::find( uniqueNeighbors.begin(),  uniqueNeighbors.end(), neighbor) ==  uniqueNeighbors.end()) {
                uniqueNeighbors.push_back(neighbor);
            }
        }

        neighborhood[i] = uniqueNeighbors;



    }
}

void ARAPMeshDeformer::
initializeWeightMatrix() {

    std::vector<std::vector<Eigen::Vector2i>> vertexEdges(V_in->rows());
    // Iterate over the vertices
    for (int i = 0; i < V_in->rows(); i++) {
        std::vector<Eigen::Vector2i> edges;

        for (int j = 0; j < F_in->rows(); j++) { // Iterate over the faces
            Eigen::Vector3i face = F_in->row(j);

            for (int k = 0; k < 3; k++) { // Iterate over the triangle
                if (face[k] == i) {
                    edges.emplace_back(face[(k + 1) % 3], face[(k + 2) % 3]);
                }
            }
        }

        vertexEdges[i] = edges;
    }
    std::cout<<"vertex edges initialized"<<std::endl;


    weightMatrix = Eigen::MatrixXf::Zero(V_in->rows(), V_in->rows());
    // Iterate over the vertices
    for (int i = 0; i < V_in->rows(); i++) {
        // Iterate over the neighbors
        for (int neighbor : neighborhood[i]) {
            float totalAngle = 0.0;

            //interate over all edges
            for (const Eigen::Vector2i& edge : vertexEdges[i]) {
                if(edge[0] == neighbor || edge[1] == neighbor){
                    float norm_bc = (V_in->row(edge[0]) - V_in->row(edge[1])).norm(); // Norm between B and C
                    float norm_ac = (V_in->row(i) - V_in->row(edge[1])).norm(); // Norm between A and C
                    float norm_ab = (V_in->row(i) - V_in->row(edge[0])).norm(); // Norm between A and B

                    // From cosine law
                    float beta = acos(((norm_ab * norm_ab) + (norm_bc * norm_bc) - (norm_ac * norm_ac)) / (2 * norm_ab * norm_bc));


                    // Add to total angle if one of the points on the edge is the current neighbor
                    totalAngle += (edge[0] == neighbor) * abs(tan(M_PI-beta));
                    totalAngle += (edge[1] == neighbor) * abs(tan(M_PI-beta));
                }


            }

            weightMatrix(i, neighbor) = abs(totalAngle) / 2;
        }

        //set weight of diagonal entry = 1
        weightMatrix(i, i) = 1.0;
    }
}

void ARAPMeshDeformer::
computeLMatrix() {
    LMatrix= Eigen::MatrixXf::Zero(V_in->rows(), V_in->rows());
    // Iterate over the vertices
    for (int i = 0; i < V_in->rows(); i++) {
        for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
            LMatrix(i, i) += weightMatrix(i, neighbor);
            LMatrix(i, neighbor) -= weightMatrix(i, neighbor);
        }
    }
}

std::vector<Eigen::Matrix3f>  ARAPMeshDeformer::
localStep(Eigen::MatrixX3f &deformedVertices) {

    std::vector<Eigen::Matrix3f> rotationMatrices(V_in->rows());

    for (int i = 0; i < V_in->rows(); i++) {
        int neighborNum = neighborhood[i].size();
        // The definitions for the matrices P, D and P_prime can be found in the paper!
        Eigen::MatrixXf P = Eigen::MatrixXf::Zero(3, neighborNum);
        Eigen::MatrixXf D = Eigen::MatrixXf::Zero(neighborNum, neighborNum);
        Eigen::MatrixXf P_prime = Eigen::MatrixXf::Zero(3, neighborNum);

        //set PDP'
        for(int j = 0; j <neighborNum;j++){
            D(j, j) = weightMatrix(i, neighborhood[i][j]);
            P.col(j) = V_in->row(i) - V_in->row(neighborhood[i][j]);
            P_prime.col(j) = deformedVertices.row(i) - deformedVertices.row(neighborhood[i][j]);
        }

        // S, the covariance matrix
        Eigen::Matrix3f S = P * D * P_prime.transpose();

        // SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::Matrix3f& U = svd.matrixU();
        const Eigen::Matrix3f& V = svd.matrixV();

        // Computation of matrix I is necessary since UV' is only orthogonal, but not necessarily a rotation matrix
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        I(2, 2) = (U * V.transpose()).determinant();

        // Add the rotation matrix R to the list
        Eigen::Matrix3f R = U * I * V.transpose();
        rotationMatrices[i] = R;
    }

    return rotationMatrices;

}
Eigen::MatrixXf  ARAPMeshDeformer::
compute_b( std::vector<Eigen::Matrix3f>  &rotationMatrices){
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(V_in->rows(), 3);

    for (int i = 0; i < V_in->rows(); i++) { // Iterate over the vertices
        // Iterate over the neighbors
        Eigen::Vector3f bPerRow =  Eigen::Vector3f(0.0,0.0,0.0);
        // need to deal with fix vertices and handle vertices
        std::vector<int>::iterator it = find(handleIndex.begin(),handleIndex.end(),i);
        if(it!= handleIndex.end()){
            //vertex index i is a handle vertex
            //just assign deformed handle to vertex
            bPerRow = this->handleManager->get_handle(it - handleIndex.begin());
        }
        else {
            for (int neighbor: neighborhood[i]) {
                bPerRow += 0.5 * weightMatrix(i, neighbor) *
                           (V_in->row(i) - V_in->row(neighbor)) *
                           (rotationMatrices[i] + rotationMatrices[neighbor]);
            }
        }

        b.row(i) = bPerRow;
    }


    return b;
}

void ARAPMeshDeformer::
globalStep(Eigen::MatrixX3f &deformedVertices,std::vector<Eigen::Matrix3f>  &rotationMatrices){
    Eigen::MatrixXf b = compute_b(rotationMatrices);
    //update L matrix for  handle points
    //solver matrix
    solver.compute(LMatrix.sparseView());
    deformedVertices = solver.solve(b);
}

float ARAPMeshDeformer::
computeEnergy(Eigen::MatrixX3f &deformedVertices,std::vector<Eigen::Matrix3f>  &rotationMatrices){
    float totalEnergy = 0;
    for (int i = 0; i < V_in->rows(); i++) { // Iterate over the vertices
        float vertexEnergy = 0;
        for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
            Eigen::Vector3f deformedPositionsDiff = deformedVertices.row(i) - deformedVertices.row(neighbor);
            Eigen::Vector3f undeformedPositionsDiff = V_in->row(i) - V_in->row(neighbor);
            vertexEnergy += weightMatrix(i, neighbor) * (deformedPositionsDiff - rotationMatrices[i] * undeformedPositionsDiff).squaredNorm();
        }
        totalEnergy += weightMatrix(i,i) * vertexEnergy;
    }
    return totalEnergy;
}
