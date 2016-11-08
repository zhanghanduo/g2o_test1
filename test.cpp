#include <iostream>
// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/types_slam3d.h>
// #include <g2o/types/slam3d/parameter_se3_offset.h>
// #include <g2o/types/slam3d/se3quat.h>
// #include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sba/types_sba.h>

using namespace std;

typedef g2o::BlockSolver_6_3 SE3BlockSolver;
typedef g2o::LinearSolverCholmod<SE3BlockSolver::PoseMatrixType> Solver;

int main(int argc, char** argv)
{
    Solver *solver = new Solver();
    solver->setBlockOrdering(false);
    SE3BlockSolver *se3blockSolver = new SE3BlockSolver(solver);
    g2o::OptimizationAlgorithmLevenberg *optimizationsolver = new g2o::OptimizationAlgorithmLevenberg(se3blockSolver);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(optimizationsolver);
    optimizer.setVerbose(true);


    size_t num = 5;
    for (size_t i = 0; i < num; i++)
    {
        g2o::VertexCam* v = new g2o::VertexCam();
        v->setId(i);
        if (i==0)
            v->setEstimate(g2o::SBACam(Eigen::Quaterniond(0,0,0,1), Eigen::Vector3d(10,10,0)));
        if (i==1)
            v->setEstimate(g2o::SBACam(Eigen::Quaterniond(0,0,0,1), Eigen::Vector3d(-1,0,0)));
            v->setFixed(true);
        if (i==2)
            v->setEstimate(g2o::SBACam(Eigen::Quaterniond(0,0,0,1), Eigen::Vector3d(0,1,0)));
            v->setFixed(true);
        if (i==3)
            v->setEstimate(g2o::SBACam(Eigen::Quaterniond(0,0,0,1), Eigen::Vector3d(1,0,0)));
            v->setFixed(true);
        if (i==4)
            v->setEstimate(g2o::SBACam(Eigen::Quaterniond(0,0,0,1), Eigen::Vector3d(0,-1,0)));
            v->setFixed(true);
        optimizer.addVertex(v);
    }

    for (size_t i = 1; i < num; ++i)
    {
        g2o::EdgeSBAScale *edge = new g2o::EdgeSBAScale();
        edge->vertices()[0] = optimizer.vertex(0);
        edge->vertices()[1] = optimizer.vertex(i);

        edge->setMeasurement(1);
        Eigen::MatrixXd information = Eigen::MatrixXd::Zero(1, 1);
        information(0,0) = 0.1;
        edge->setInformation(information.inverse());

        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    for (size_t i = 0; i < num; i++)
    {
        g2o::VertexCam *v = dynamic_cast<g2o::VertexCam *> (optimizer.vertex(i));
        cout << "vertex id " << i << " Pose=" << endl << v->estimate()<< endl;
    }


    return 0;
}

// int main( int argc, char** argv )
// {
//     typedef g2o::BlockSolver_6_3 SlamBlockSolver;
//     typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

//     SlamLinearSolver *linearSolver = new SlamLinearSolver();
//     linearSolver->setBlockOrdering(false);
//     SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
//     g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
//     // g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);


//     g2o::SparseOptimizer optimizer;
//     optimizer.setAlgorithm(solver);
//     optimizer.setVerbose( true );

//     size_t num = 100;
//     for (size_t i = 0; i < num; i++)
//     {
//         g2o::VertexSE3 *v = new g2o::VertexSE3();
//         v->setId(i);
//         v->setEstimate(Eigen::Isometry3d::Identity());
//         if (i==0)
//             v->setFixed(true);
//         optimizer.addVertex( v );
//     }


// //    g2o::ParameterSE3Offset* parameterse3offset = new g2o::ParameterSE3Offset();
// //    parameterse3offset->setOffset(g2o::Isometry3D::Identity());
// //    parameterse3offset->setId(0);
// //    optimizer.addParameter(parameterse3offset);

//     vector<g2o::EdgeSE3 *> edges;
//     for (size_t i = 0; i < num-1; ++i)
//     {
//         g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
//         edge->vertices()[0] = optimizer.vertex(i);
//         edge->vertices()[1] = optimizer.vertex(i + 1);
// //        edge->setVertex(0, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex(i)));
// //        edge->setVertex(1, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex((i + 1) % num)));

//         Eigen::Isometry3d pose;
//         pose.setIdentity();
//         pose.translate(g2o::Vector3D(1, 0, 0) + 0.1 * g2o::Vector3D::Random());

//         cout << "measurement id " << i << " Pose=" << endl << pose.matrix() << endl;
//         edge->setMeasurement(pose);
//         Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) * 0.01;
//         information(3,3) = 0.0001;
//         information(4,4) = 0.0001;
//         information(5,5) = 0.0001;

//         edge->setInformation(information.inverse());
//         cout << "information id " << i << " information=" << endl << information.inverse() << endl;

// //        edge->setParameterId(0, parameterse3offset->id());
//         edge->setRobustKernel( new g2o::RobustKernelHuber() );
//         optimizer.addEdge( edge );
//         edges.push_back(edge);
//     }

//     g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
//     edge->vertices()[0] = optimizer.vertex(0);
//     edge->vertices()[1] = optimizer.vertex(99);
//     Eigen::Isometry3d pose;
//     pose.setIdentity();
//     pose.translate(g2o::Vector3D(99, 0, 0));// + 0.01 * g2o::Vector3D::Random());
//     edge->setMeasurement(pose);
//     Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) * 0.01;
//     information(3,3) = 0.0001;
//     information(4,4) = 0.0001;
//     information(5,5) = 0.0001;
//     edge->setInformation(information.inverse());
//     optimizer.addEdge( edge );
//     edges.push_back(edge);

//     optimizer.save( "result_before.g2o" );
//     optimizer.setVerbose(true);
//     optimizer.initializeOptimization();
//     optimizer.optimize(100);
//     optimizer.save( "result_after.g2o" );


//     for (size_t i = 0; i < num; i++)
//     {
//         g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(i));
//         Eigen::Isometry3d pos = v->estimate();
//         cout << "vertex id " << i << " Pose=" << endl << pos.matrix() << endl;
//     }

//     cout <<"vertex done."<<endl;

//     int inliers = 0;
//     for ( auto e:edges )
//     {
//         e->computeError();
//         cout<<"error = "<<e->chi2()<<endl;
//         if ( e->chi2() > 1 )
//         {
//             cout<<"error = "<<e->chi2()<<endl;
//         }
//         else
//         {
//             inliers++;
//         }
//     }

//     cout << "inliers in total points: " << inliers << "/" << edges.size() << endl;
//     return 0;
// }
