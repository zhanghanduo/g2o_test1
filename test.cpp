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
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;


int main( int argc, char** argv )
{
    vector<cv::Point2f> pts1, pts2;

    g2o::SparseOptimizer    optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( true );


    for (int i = 0; i < 3; i++)
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true );
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }

    vector<g2o::EdgeSE3Expmap *> edges;
    for (size_t i = 0; i < 3; ++i)
    {
        g2o::EdgeSE3Expmap *edge = new g2o::EdgeSE3Expmap();
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap *>   (optimizer.vertex(i)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>   (optimizer.vertex((i + 1) % 3)));
        edge->setMeasurement(g2o::SE3Quat());
        edge->setInformation(Eigen::MatrixXd::Identity(6, 6));
        edge->setParameterId(0, 0);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }


    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(10);


    for (size_t i = 0; i < edges.size(); i++)
    {
        g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *> (optimizer.vertex(i));

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
    optimizer.save("ba.g2o");
    return 0;
}