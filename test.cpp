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

using namespace std;


int main( int argc, char** argv )
{
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    // g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);


    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose( true );

    size_t num = 4;
    for (size_t i = 0; i < num; i++)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(i);
        v->setEstimate(Eigen::Isometry3d::Identity());
        Eigen::Isometry3d node=Eigen::Isometry3d::Identity();
        switch (i) {
            case 0:
                v->setFixed(true);
                v->setEstimate(Eigen::Isometry3d::Identity());
                break;
            case 1:
                node.translate(g2o::Vector3D(1.1, 0, 0));
                v->setEstimate(node);
                break;
            case 2:
                node.translate(g2o::Vector3D(1.1, 1.1, 0));
                v->setEstimate(node);
                break;
            case 3:
                node.translate(g2o::Vector3D(0, 1.1, 0));
                v->setEstimate(node);
                break;

        }
        optimizer.addVertex( v );
    }


//    g2o::ParameterSE3Offset* parameterse3offset = new g2o::ParameterSE3Offset();
//    parameterse3offset->setOffset(g2o::Isometry3D::Identity());
//    parameterse3offset->setId(0);
//    optimizer.addParameter(parameterse3offset);

    vector<g2o::EdgeSE3 *> edges;
    for (size_t i = 0; i < num; ++i)
    {
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        edge->vertices()[0] = optimizer.vertex(i);
        edge->vertices()[1] = optimizer.vertex((i + 1) % num);
//        edge->setVertex(0, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex(i)));
//        edge->setVertex(1, dynamic_cast<g2o::VertexSE3 *>   (optimizer.vertex((i + 1) % num)));

        Eigen::Isometry3d pose;
        pose.setIdentity();
        switch (i % num) {
            case 0:
                pose.translate(g2o::Vector3D(1, 0, 0) - 0.1 * g2o::Vector3D::Random());
                break;
            case 1:
                pose.translate(g2o::Vector3D(0, 1, 0) - 0.1 * g2o::Vector3D::Random());
                break;
            case 2:
                pose.translate(g2o::Vector3D(-1, 0, 0) - 0.1 * g2o::Vector3D::Random());
                break;
            case 3:
                pose.translate(g2o::Vector3D(0, -1, 0) - 0.1 * g2o::Vector3D::Random());
                break;

        }
        cout << "measurement id " << i << " Pose=" << endl << pose.matrix() << endl;
        edge->setMeasurement(pose);
        Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6) * 0.01;
        information(3,3) = 0.0001;
        information(4,4) = 0.0001;
        information(5,5) = 0.0001;

        edge->setInformation(information.inverse());
        cout << "information id " << i << " information=" << endl << information.inverse() << endl;

//        edge->setParameterId(0, parameterse3offset->id());
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    optimizer.save( "result_before.g2o" );
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    optimizer.save( "result_after.g2o" );


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
            inliers++;
        }
    }

    cout << "inliers in total points: " << inliers << "/" << edges.size() << endl;
    return 0;
}