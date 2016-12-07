//
// Created by ugv on 12/7/16.
//

#ifndef G2O_BA_EXAMPLE_ORB_EDGE_H
#define G2O_BA_EXAMPLE_ORB_EDGE_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/se3_ops.h"
#include "g2o/types/slam3d/se3quat.h"
//#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/types_sba.h"
#include <Eigen/Geometry>

namespace g2o {
    namespace orb_edge {
        void init();
    }

    typedef Eigen::Matrix<double, 6, 6, Eigen::ColMajor> Matrix6d;

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

VertexSE3Expmap();

bool read(std::istream& is);

bool write(std::ostream& os) const;

virtual void setToOriginImpl() {
    _estimate = SE3Quat();
}

virtual void oplusImpl(const double* update_)  {
    Eigen::Map<const Vector6d> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
}
};


/**
 * \brief 6D edge between two Vertex6
 */
class EdgeSE3Expmap : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
EdgeSE3Expmap();

bool read(std::istream& is);

bool write(std::ostream& os) const;

void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

    SE3Quat C(_measurement);
    SE3Quat error_= v2->estimate().inverse()*C*v1->estimate();
    _error = error_.log();
}

virtual void linearizeOplus();
};

class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

EdgeSE3ProjectXYZ();

bool read(std::istream& is);

bool write(std::ostream& os) const;

void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2D obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(v2->estimate()));
}

bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
}


virtual void linearizeOplus();

Vector2D cam_project(const Vector3D & trans_xyz) const;

double fx, fy, cx, cy;
};


class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3D, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

EdgeStereoSE3ProjectXYZ();

bool read(std::istream& is);

bool write(std::ostream& os) const;

void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector3D obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
}

bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
}


virtual void linearizeOplus();

Vector3D cam_project(const Vector3D & trans_xyz, const float &bf) const;

double fx, fy, cx, cy, bf;
};

class EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2D, VertexSE3Expmap>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

EdgeSE3ProjectXYZOnlyPose(){}

bool read(std::istream& is);

bool write(std::ostream& os) const;

void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector2D obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(Xw));
}

bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
}


virtual void linearizeOplus();

Vector2D cam_project(const Vector3D & trans_xyz) const;

Vector3D Xw;
double fx, fy, cx, cy;
};


class EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3D, VertexSE3Expmap>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

EdgeStereoSE3ProjectXYZOnlyPose(){}

bool read(std::istream& is);

bool write(std::ostream& os) const;

void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector3D obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(Xw));
}

bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
}


virtual void linearizeOplus();

Vector3D cam_project(const Vector3D & trans_xyz) const;

Vector3D Xw;
double fx, fy, cx, cy, bf;
};

class EdgeStereoSE3ProjectXYZOnlyPose_rot_known: public  BaseUnaryEdge<3, Vector3D, VertexSBAPointXYZ>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

EdgeStereoSE3ProjectXYZOnlyPose_rot_known(){}

bool read(std::istream& is);

bool write(std::ostream& os) const;

void computeError()  {
    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector3D obs(_measurement);
    _error = obs - cam_project(v1->estimate() + rot * Xw);
}

bool isDepthPositive() {
    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate() + rot * Xw)(2)>0.0;
}


virtual void linearizeOplus();

Vector3D cam_project(const Vector3D & trans_xyz) const;

Vector3D Xw;
// Eigen::Quaterniond rot;
Matrix3D rot;
double fx, fy, cx, cy, bf;
};

class EdgeSE3ProjectXYZOnlyPose_rot_known: public  BaseUnaryEdge<2, Vector2D, VertexSBAPointXYZ>{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

EdgeSE3ProjectXYZOnlyPose_rot_known(){}

bool read(std::istream& is);

bool write(std::ostream& os) const;

void computeError()  {
    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2D obs(_measurement);
    _error = obs-cam_project(v1->estimate() + rot * Xw);
}

bool isDepthPositive() {
    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate() + rot * Xw)(2)>0.0;
}


virtual void linearizeOplus();

Vector2D cam_project(const Vector3D & trans_xyz) const;

Vector3D Xw;
// Eigen::Quaterniond rot;
Matrix3D rot;
double fx, fy, cx, cy;
};

} // end namespace

#endif //G2O_BA_EXAMPLE_ORB_EDGE_H

