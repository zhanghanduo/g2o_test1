// This example consists of a single static target which sits in one
// place and does not move; in effect it has a "GPS" which measures
// its position

#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include "g2o/core/optimization_algorithm_levenberg.h"
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/stuff/sampler.h>
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "orb_edge.h"
//#include "g2o/types/sba/types_six_dof_expmap.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

int main()
{
  // Set up the optimiser
  SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  unsigned int numMeasurements = 7;
  // Create the block solver - the dimensions are specified because
  // 3D observations marginalise to a 3D estimate
//  typedef BlockSolver<BlockSolverTraits<3, 3> > BlockSolver_3_3;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
          = new LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();
//      = new LinearSolverEigen<BlockSolver_3_3::PoseMatrixType>();
  BlockSolver_6_3* blockSolver
      = new BlockSolver_6_3(linearSolver);
//  OptimizationAlgorithmGaussNewton* solver
//    = new OptimizationAlgorithmGaussNewton(blockSolver);
    OptimizationAlgorithmLevenberg* solver
            = new OptimizationAlgorithmLevenberg(blockSolver);
  optimizer.setAlgorithm(solver);

  // Sample the actual location of the target of 7 points in world coordinate
  std::vector<Vector3d> truePoint;
  truePoint.reserve(numMeasurements);
  Vector3d truePoint1(5,3.2,101);
  Vector3d truePoint2(-7.5,10,58);
  Vector3d truePoint3(8,-9,87);
  Vector3d truePoint4(12,1,43);
  Vector3d truePoint5(-14,-2,49);
  Vector3d truePoint6(-3,2,35);
  Vector3d truePoint7(-8,-4,108);
  truePoint.push_back(truePoint1);
  truePoint.push_back(truePoint2);
  truePoint.push_back(truePoint3);
  truePoint.push_back(truePoint4);
  truePoint.push_back(truePoint5);
  truePoint.push_back(truePoint6);
  truePoint.push_back(truePoint7);

  // observations on image
  std::vector<cv::KeyPoint> mKeypoint;
  mKeypoint.reserve(numMeasurements);
  mKeypoint[0].pt.x = 255;
  mKeypoint[0].pt.y = 336;
  mKeypoint[1].pt.x = 151;
  mKeypoint[1].pt.y = 425;
  mKeypoint[2].pt.x = 278;
  mKeypoint[2].pt.y = 266;
  mKeypoint[3].pt.x = 439;
  mKeypoint[3].pt.y = 337;
  mKeypoint[4].pt.x = 37;
  mKeypoint[4].pt.y = 304;
  mKeypoint[5].pt.x = 150;
  mKeypoint[5].pt.y = 53;
  mKeypoint[6].pt.x = 194;
  mKeypoint[6].pt.y = 302;
  // Construct vertex which corresponds to the actual point of the target
  g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
  Matrix3d R_init = Matrix3D::Identity();
  Vector3d T_init(0.2,-0.5,-30);
  vSE3->setEstimate(SE3Quat(R_init,T_init));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  std::vector<EdgeSE3ProjectXYZOnlyPose*> edge_mono;
  edge_mono.reserve(numMeasurements);

  // Now generate some noise corrupted measurements; for simplicity
  // these are uniformly distributed about the true target. These are
  // modelled as a unary edge because they do not like to, say,
  // another node in the map.

  const double noiseLimit = sqrt(12.);
  const double noiseSigma = noiseLimit*noiseLimit / 12.0;
  const float deltaMono = sqrt(5.991);
  for (unsigned int i = 0; i < numMeasurements; i++)
    {
      Matrix<double,2,1> obs;
      const cv::KeyPoint &key = mKeypoint[i];
      obs << key.pt.x, key.pt.y;
      g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
//      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
      e->setVertex(0, dynamic_cast<VertexSE3Expmap*>(optimizer.vertex(0)));

      e->setMeasurement(obs);
//      e->setInformation(Matrix2d::Identity() / noiseSigma);
      e->setInformation(Matrix2d::Identity() );
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      e->setRobustKernel(rk);
      rk->setDelta(deltaMono);
      e->fx = 400;
      e->fy = 400;
      e->cx = 230;
      e->cy = 320;
//      e->rot = Quaterniond(0,0,0,1);
//      e->rot = Matrix3D::Identity();
      e->Xw = truePoint[i];
      optimizer.addEdge(e);
      edge_mono.push_back(e);
    }

//  Trans3->setEstimate(init);
  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(10);
  for(size_t i = 0, iend=edge_mono.size();i<iend; i++){
    g2o::EdgeSE3ProjectXYZOnlyPose* e = edge_mono[i];
    e->computeError();
//    e->setLevel(0);
    e->setRobustKernel(0);
  }

  VertexSE3Expmap* recov = static_cast<VertexSE3Expmap*>(optimizer.vertex(0));
  SE3Quat recov_3d = recov->estimate();


//  std::cout << "true point=\n" << truePoint << std::endl;

  cout <<  "computed translation estimate=\n" << recov_3d.translation()(0) << " and " <<recov_3d.translation()(1)<< " and " << recov_3d.translation()(2) << endl;
  cout << "rotation estimate=\n" << "x: " << recov_3d.rotation().x() << "y: " << recov_3d.rotation().y()<< "z: " << recov_3d.rotation().z()<< "w: " << recov_3d.rotation().w()  << endl;
  //position->setMarginalized(true);

  // SparseBlockMatrix<MatrixXd> spinv;
  //
  // optimizer.computeMarginals(spinv, position);



  //optimizer.solver()->computeMarginals();

  // covariance
  //
  // cout << "covariance\n" << spinv << endl;
  //
  // cout << spinv.block(0,0) << endl;

}
