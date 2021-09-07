#define _USE_MATH_DEFINES
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block_factory.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/all_types.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include <srrg_solver/solver_core/iteration_algorithm_lm.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;
const std::string example_folder(SRRG2_SOLVER_EXAMPLE_FOLDER);

const float _imu_freq = 300; // ldg assuming imu @ 100 Hz

void generateImuMotion(std::vector<Isometry3f>& poses_,
                       std::vector<Vector3f>& f_velocities_,
                       std::vector<Vector3f>& acc_,
                       std::vector<Vector3f>& gyro_,
                       std::vector<float>& delta_times_) {
  const float T  = 5.0; // ldg total time of trajectory [s]
  const float dt = 1.0 / _imu_freq;

  Vector3f a = Vector3f(0.1f, 0.1f, 0.1f);
  Vector3f w = Vector3f(0.1, 0.1, 0.1);
  Vector3f v = Vector3f(0.f, 0.f, 0.f);
  Matrix3f R = Matrix3f::Identity();
  Vector3f p = Vector3f::Zero();

  size_t size_vec = static_cast<size_t>(T * _imu_freq);

  poses_.resize(size_vec);
  f_velocities_.resize(size_vec);
  acc_.resize(size_vec);
  gyro_.resize(size_vec);
  delta_times_.resize(size_vec);

  f_velocities_.at(0).setZero();
  poses_.at(0).setIdentity();

  std::fill(acc_.begin(), acc_.end(), a);
  std::fill(gyro_.begin(), gyro_.end(), w);
  std::fill(delta_times_.begin(), delta_times_.end(), dt);
  for (size_t i = 1; i < size_vec; ++i) {
    const Vector3f dtheta = w * dt;
    p += v * dt + 0.5f * R * a * dt * dt;
    v += R * a * dt;
    R *= expMap(dtheta);
    f_velocities_.at(i)        = v;
    poses_.at(i)               = Isometry3f::Identity();
    poses_.at(i).linear()      = R;
    poses_.at(i).translation() = p;
  }
}

int main(int argc, char* argv[]) {
  std::vector<Isometry3f> poses;
  std::vector<Vector3f> f_vel, acc, gyro;
  std::vector<float> dts;
  generateImuMotion(poses, f_vel, acc, gyro, dts);

  // ofstream dumper;
  // dumper.open("/home/ldg/Desktop/data.txt");
  // dumper << "# timestamp tx ty tz v a";
  // float time = 0.0;
  // for (size_t i = 0; i < poses.size(); ++i) {
  //   time += dts.at(i);
  //   const auto& t = poses.at(i).translation();
  //   dumper << time << " " << t.x() << " " << t.y() << " " << t.z() << " " << f_vel.at(i).norm()
  //          << " " << acc.at(i).norm() << "\n";
  // }
  // dumper.close();

  Solver solver;
  // solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(5);
  IterationAlgorithmBasePtr alg(new IterationAlgorithmLM);
  solver.param_algorithm.setValue(alg);
  FactorGraphPtr graph(new FactorGraph);

  using VarPoseType = VariableSE3QuaternionRightAD;
  using VarVelType  = VariablePoint3AD;
  using FactorType  = SE3ImuErrorFactor;

  PreintegratedImuMeasurement imu_measurement;

  VarPoseType* init_pose_var = new VarPoseType();
  init_pose_var->setEstimate(Isometry3f::Identity());
  init_pose_var->setGraphId(0);
  init_pose_var->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(init_pose_var));

  VarVelType* init_vel_var = new VarVelType();
  init_vel_var->setEstimate(Vector3f::Zero());
  init_vel_var->setGraphId(1);
  init_vel_var->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(init_vel_var));

  const int integration_interval = static_cast<int>(_imu_freq / 8);
  size_t graph_id                = 2;
  for (size_t i = 1; i < acc.size(); ++i) {
    const auto acc_noisy  = acc.at(i) + Vector3f::Random() * 1e-2;
    const auto gyro_noisy = gyro.at(i) + Vector3f::Random() * 1e-3;
    imu_measurement.preIntegrate(acc_noisy, gyro_noisy, dts.at(i));
    if (i % integration_interval != 0) {
      continue;
    }

    VarPoseType* curr_pose_var = new VarPoseType();
    curr_pose_var->setEstimate(Isometry3f::Identity());
    curr_pose_var->setGraphId(graph_id++);
    graph->addVariable(VariableBasePtr(curr_pose_var));

    VarVelType* curr_vel_var = new VarVelType();
    curr_vel_var->setGraphId(graph_id++);
    curr_vel_var->setEstimate(Vector3f::Zero());
    graph->addVariable(VariableBasePtr(curr_vel_var));

    FactorType* imu_factor = new FactorType();
    imu_factor->setVariableId(0, graph_id - 4);
    imu_factor->setVariableId(1, graph_id - 3);
    imu_factor->setVariableId(2, curr_pose_var->graphId());
    imu_factor->setVariableId(3, curr_vel_var->graphId());
    imu_factor->setMeasurement(imu_measurement);

    graph->addFactor(FactorBasePtr(imu_factor));
    imu_measurement.reset();

    solver.setGraph(graph);
    solver.compute();
  }

  using FactorTypeGeodesic        = SE3PosePoseGeodesicErrorFactor;
  FactorTypeGeodesic* odom_factor = new FactorTypeGeodesic();
  odom_factor->setVariableId(0, 0);
  odom_factor->setVariableId(1, graph_id - 2);
  Vector3f angle_noise    = Vector3f::Random() * 1e-3;
  Vector3f t_noise        = Vector3f::Random() * 1e-2;
  Isometry3f iso_noise    = Isometry3f::Identity();
  iso_noise.linear()      = geometry3d::a2r(angle_noise);
  iso_noise.translation() = t_noise;
  const auto Z_odom       = poses.back() * iso_noise;
  odom_factor->setMeasurement(Z_odom);
  // odom_factor->setInformationMatrix(odom_information);
  graph->addFactor(FactorBasePtr(odom_factor));

  solver.setGraph(graph);
  solver.compute();
  std::cerr << solver.iterationStats() << std::endl;

  const auto& estimate_pose  = static_cast<VarPoseType*>(graph->variable(graph_id - 2))->estimate();
  const auto& gt_pose        = poses.back();
  const auto diff_T          = estimate_pose.inverse() * gt_pose;
  const Matrix3f diff_angles = diff_T.linear();

  std::cerr << "last estimate: \n" << estimate_pose.matrix() << std::endl;
  std::cerr << "last gt pose: \n" << gt_pose.matrix() << std::endl;
  std::cerr << "error translation [m]: " << diff_T.translation().transpose() << std::endl;
  std::cerr << "error rotation [rad]: " << geometry3d::r2a(diff_angles).transpose() << std::endl;
}
