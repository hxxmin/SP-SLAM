#pragma once

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Surfels.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <algorithm> // std::swap

#include "Converter.h"

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

using namespace SP_SLAM;

namespace g2o
{

// variable: point(3), normal(3) ...
class VertexSurfel : public BaseVertex<6, Surfel> // NOTE  this vertex stores object pose to world
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSurfel(){};

    virtual void setToOriginImpl() { _estimate = Surfel(); }

    virtual void oplusImpl(const double *update_)
    {
        Eigen::Map<const Vector6d> update(update_);
        setEstimate(_estimate.exp_update(update));
    }

    virtual bool read(std::istream &is)
    {
        Vector6d est;
        for (int i = 0; i < 6; i++)
            is >> est[i];
        Surfel sf;
        sf.fromMinimalVector(est);
        setEstimate(sf);
        return true;
    }

    virtual bool write(std::ostream &os) const
    {
        Vector6d lv = _estimate.toMinimalVector();
        for (int i = 0; i < lv.rows(); i++)
        {
            os << lv[i] << " ";
        }
        return os.good();
    }
};

// camera - surfel 3D error
class EdgeSE3Surfel : public BaseBinaryEdge<6, Surfel, VertexSE3Expmap, VertexSurfel>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3Surfel(){};

    virtual bool read(std::istream &is)
    {
        return true;
    };

    virtual bool write(std::ostream &os) const
    {
        return os.good();
    };

    void computeError()
    {
        const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]); //  camera pose (world coordinate)
        const VertexSurfel *surfelVertex = static_cast<const VertexSurfel *>(_vertices[1]);     //  map surfel (world coordinate)

        SE3Quat pose_cam = SE3Vertex->estimate().inverse(); // inverse is right...
        Surfel global_surfel = surfelVertex->estimate();

        cv::Mat cam_pose_Twc = Converter::toCvMat(pose_cam);
        Surfel meas_sf_global = _measurement.transform_L2G(cam_pose_Twc);

        _error =  global_surfel.surfel_plane_error(meas_sf_global);

    }

    double get_error_norm() // for debug
    {
        const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]); //  world to camera pose
        const VertexSurfel *surfelVertex = static_cast<const VertexSurfel *>(_vertices[1]);     //  object pose to world

        SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
        cv::Mat cam_pose_Twc_cvMat = Converter::toCvMat(cam_pose_Twc);

        Surfel global_surfel = surfelVertex->estimate();
        Surfel esti_global_surfel = _measurement.transform_L2G(cam_pose_Twc_cvMat);

        return global_surfel.surfel_log_error(esti_global_surfel).norm();
    }
};

} // namespace g2o
