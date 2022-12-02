#pragma once

#include <string>
#include <vector>

#include "pch.hpp"

class HandleManager final {
public:
    size_t add_handle(Eigen::RowVector3f init_pos) {
        this->handles_rest.conservativeResize(this->handles_rest.rows() + 1, Eigen::NoChange_t());
        auto idx = this->handles_rest.rows() - 1;
        this->handles_rest.row(idx) << init_pos;
        return idx;
    }
    void update_handle(int idx, Eigen::RowVector3f new_pos) { this->handles.row(idx) = new_pos; }
    void update_handle_rest(int idx, Eigen::RowVector3f new_pos) { this->handles_rest.row(idx) = new_pos; }
    void lock_rest_handles() {
        size_t last_row = this->handles.rows() - 1;
        this->handles.conservativeResizeLike(this->handles_rest); /* matrix copy */
        for (size_t i = last_row; i < get_handles_count(); ++i) {
            this->handles.row(i) = this->handles_rest.row(i);
        }
    }

    Eigen::RowVector3f get_handle(int idx) const { return this->handles.row(idx); }
    Eigen::RowVector3f get_handle_rest(int idx) const { return this->handles_rest.row(idx); }
    float* get_handle_ptr(int idx) { return this->handles.row(idx).data(); }
    float* get_handle_rest_ptr(int idx) { return this->handles_rest.row(idx).data(); }
    const Eigen::Matrix<float, -1, 3, Eigen::RowMajor>& get_handles() const { return this->handles; }
    const Eigen::Matrix<float, -1, 3, Eigen::RowMajor>& get_handles_rest() const { return this->handles_rest; }
    Eigen::Matrix<float, -1, 3, Eigen::RowMajor>& get_handles() { return this->handles; }
    Eigen::Matrix<float, -1, 3, Eigen::RowMajor>& get_handles_rest() { return this->handles_rest; }
    size_t get_handles_count() const { return handles.rows(); }
    size_t get_handles_rest_count() const { return handles_rest.rows(); }
    void reset_handles() { this->handles = this->handles_rest; }
    void clear_handles() {
        this->handles = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>();
        this->handles_rest = this->handles;
    }
protected:
    Eigen::Matrix<float, -1, 3, Eigen::RowMajor> handles;
    Eigen::Matrix<float, -1, 3, Eigen::RowMajor> handles_rest;
};

class AnyParams final {
public:
    template<typename T> void init(std::string key, T val) {
        if (this->params.find(key) == this->params.end()) {
            this->set<T>(key, val);
        }
    }
    template<typename T> void set(std::string key, T val) { this->params[key] = val; }
    template<typename T> T& get(std::string key) { return *boost::any_cast<T>(&this->params[key]); }

    auto begin() { return this->params.begin(); }
    auto end() { return this->params.end(); }
protected:
    tsl::ordered_map<std::string, boost::any> params;
};

class AbstractMeshDeformer {
public:
    virtual ~AbstractMeshDeformer() {};

    void bind_data(
        AnyParams* params = nullptr,
        const HandleManager* handleManager = nullptr
    ) {
        if (params) {
            this->init_params(*params);
            this->params = params;
        }
        if (handleManager) {
            this->handleManager = handleManager;
        }
    }

    virtual void init_params(AnyParams& params) {};
    virtual void on_load_mesh(
        const Eigen::MatrixX3f& V_in,
        const Eigen::MatrixX3i& F_in,
        const igl::opengl::ViewerData& extras
    ) {};
    virtual void on_lock_rest_handles() {};
    virtual void deform_mesh(Eigen::MatrixX3f& V_out) = 0;

protected:
    const HandleManager* handleManager = nullptr;
    AnyParams* params = nullptr;
};
