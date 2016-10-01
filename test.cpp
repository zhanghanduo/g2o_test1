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


    for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true );
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }

//    for ( size_t i=0; i<pts1.size(); i++ )
//    {
//        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
//        v->setId( 2 + i );
////        double z = 1;
////        double x = ( pts1[i].x - cx ) * z / fx;
////        double y = ( pts1[i].y - cy ) * z / fy;
//        v->setMarginalized(true);
////        v->setEstimate( Eigen::Vector3d(x,y,z) );
//        optimizer.addVertex( v );
//    }


    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }


    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose="<<endl<<pose.matrix()<<endl;

    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
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

    cout<<"inliers in total points: "<<inliers<<"/"<<pts1.size()+pts2.size()<<endl;
    optimizer.save("ba.g2o");
    return 0;
}