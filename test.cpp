#include <iostream>
// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
using namespace std;
using namespace g2o;
//using namespace g2o::types_six_dof_expmap;

int main( int argc, char** argv )
{
    g2o::SparseOptimizer    optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
    g2o::OptimizationAlgorithmGaussNewton*  algorithm = new g2o::OptimizationAlgorithmGaussNewton( block_solver );
    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( true );

    size_t num = 4;
    for (int i = 0; i < num; i++)
    {
        VertexSE3* v = new VertexSE3();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true );
        v->setEstimate( g2o::Isometry3D::Identity() );
        optimizer.addVertex( v );
    }

    vector<g2o::EdgeSE3 *> edges;
    for (size_t i = 0; i < num; ++i)
    {
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex(i)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex((i + 1) % num)));

//        Isometry3D
        g2o::Isometry3D *pose = new g2o::Isometry3D();
        pose->setIdentity();
//        pose->rotation() = Eigen::Matrix3d::Identity();
        switch (i % num)
        {
            case 0:
                pose->translation() = g2o::Vector3D(1, 0, 0) + 0.1 * g2o::Vector3D::Random();
//                pose->setTranslation(g2o::Vector3D(1, 0, 0) + 0.1 * g2o::Vector3D::Random());
//                cout <<i<<" "<<pose->toVector()<<endl;
                break;
            case 1:
                pose->translation() = g2o::Vector3D(0, 1, 0) + 0.1 * g2o::Vector3D::Random();

//                pose->setTranslation(g2o::Vector3D(0, 1, 0) + 0.1 * g2o::Vector3D::Random());
                break;
            case 2:
                pose->translation() = g2o::Vector3D(-1, 0, 0) + 0.1 * g2o::Vector3D::Random();

//                pose->setTranslation(g2o::Vector3D(-1, 0, 0) + 0.1 * g2o::Vector3D::Random());
                break;
            case 3:
                pose->translation() = g2o::Vector3D(0, -1, 0) + 0.1 * g2o::Vector3D::Random();

//                pose->setTranslation(g2o::Vector3D(0, -1, 0) + 0.1 * g2o::Vector3D::Random());
                break;
            case 4:
                pose->translation() = g2o::Vector3D(-1, 1, 0) + 0.1 * g2o::Vector3D::Random();
                edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap *>   (optimizer.vertex(1)));
                edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>   (optimizer.vertex(3)));
                break;
            case 5:
                pose->translation() = g2o::Vector3D(1, 1, 0) + 0.1 * g2o::Vector3D::Random();
                edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap *>   (optimizer.vertex(0)));
                edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>   (optimizer.vertex(2)));
        }
        cout << "measurement id " << i << " Pose=" << endl << pose->matrix() << endl;
        edge->setMeasurement(*pose);
        edge->setInformation(Eigen::MatrixXd::Identity(6, 6)*0.01);
        edge->setParameterId(0, 0);
        edge->setRobustKernel( new g2o::RobustKernelTukey() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }


    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);


    for (size_t i = 0; i < edges.size(); i++)
    {
        g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(i));
        Eigen::Isometry3d pos = v->estimate();
        cout << "vertex id " << i << " Pose=" << endl << pos.matrix() << endl;
    }

    int inliers = 0;
    for ( auto e:edges )
    {
        e->computeError();
        if ( e->chi2() > 1 )
        {
            cout<<"error = "<<e->chi2()<<endl;
        }
        else
        {
            cout<<"error = "<<e->chi2()<<endl;
            inliers++;
        }
    }

    cout << "inliers in total points: " << inliers << "/" << edges.size() << endl;
    optimizer.save("ba.g2o");
    return 0;
}