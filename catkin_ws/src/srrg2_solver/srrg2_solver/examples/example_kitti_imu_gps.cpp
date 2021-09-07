#include <iostream>
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

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

struct KittiCalibration {
  double body_ptx;
  double body_pty;
  double body_ptz;
  double body_prx;
  double body_pry;
  double body_prz;
  double accelerometer_sigma;
  double gyroscope_sigma;
  double integration_sigma;
  double accelerometer_bias_sigma;
  double gyroscope_bias_sigma;
  double average_delta_t;
};

struct ImuMeasurement {
  double time;
  double dt;
  Vector3f accelerometer;
  Vector3f gyroscope; // omega
};

struct GpsMeasurement {
  double time;
  Vector3f position; // x,y,z
};

void loadKittiData(KittiCalibration& kitti_calibration,
                   vector<ImuMeasurement>& imu_measurements,
                   vector<GpsMeasurement>& gps_measurements);

int main() {
  KittiCalibration kitti_calibration;
  vector<ImuMeasurement> imu_measurements;
  vector<GpsMeasurement> gps_measurements;
  loadKittiData(kitti_calibration, imu_measurements, gps_measurements);

  Solver solver;
  // solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(15);
  IterationAlgorithmBasePtr alg(new IterationAlgorithmLM);
  solver.param_algorithm.setValue(alg);
  FactorGraphPtr graph(new FactorGraph);

  using VarPoseImuType = VariableSE3QuaternionRightAD;
  using VarVelImuType  = VariablePoint3AD;
  using VarPoseGpsType = VariablePoint3AD;
  using FactorImuType  = SE3ImuErrorFactor;
  using FactorGpsType  = GpsErrorFactorAD;

  // initialization

  const Vector3f& init_gps_pose = gps_measurements.at(0).position;
  Isometry3f init_pose          = Isometry3f::Identity();
  init_pose.translation()       = init_gps_pose;

  VarPoseImuType* init_pose_var = new VarPoseImuType();
  init_pose_var->setEstimate(init_pose);
  init_pose_var->setGraphId(0);
  init_pose_var->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(init_pose_var));

  VarVelImuType* init_vel_var = new VarVelImuType();
  init_vel_var->setEstimate(Vector3f::Zero());
  init_vel_var->setGraphId(1);
  // init_vel_var->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(init_vel_var));

  size_t graph_id = 2;

  PreintegratedImuMeasurement imu_integrator;
  size_t j = 0;
  for (size_t i = 1; i < gps_measurements.size(); ++i) {
    const double t_previous = gps_measurements.at(i - 1).time;
    const double gps_time   = gps_measurements.at(i).time;
    while (j < imu_measurements.size() && imu_measurements.at(j).time <= gps_time) {
      if (imu_measurements.at(j).time >= t_previous) {
        imu_integrator.preIntegrate(imu_measurements.at(j).accelerometer,
                                    imu_measurements.at(j).gyroscope,
                                    imu_measurements.at(j).dt);
      }

      j++;
    }

    // order of var indeces
    // 1 pose from imu
    // 2 vel from imu
    // 3 pose to imu
    // 4 vel to imu
    // 5 pose gps

    const Vector3f curr_gps_pose = gps_measurements.at(i).position;
    Isometry3f curr_pose         = Isometry3f::Identity();
    curr_pose.translation()      = curr_gps_pose;

    VarPoseImuType* curr_pose_var = new VarPoseImuType();
    curr_pose_var->setEstimate(curr_pose);
    curr_pose_var->setGraphId(graph_id++);
    graph->addVariable(VariableBasePtr(curr_pose_var));

    VarVelImuType* curr_vel_var = new VarVelImuType();
    curr_vel_var->setGraphId(graph_id++);
    const Vector3f prev_vel =
      static_cast<VarVelImuType*>(graph->variable(graph_id - 4))->estimate();
    curr_vel_var->setEstimate(prev_vel);
    graph->addVariable(VariableBasePtr(curr_vel_var));

    FactorImuType* imu_factor = new FactorImuType();
    imu_factor->setVariableId(0, graph_id - 5);
    imu_factor->setVariableId(1, graph_id - 4); // prev vel
    imu_factor->setVariableId(2, curr_pose_var->graphId());
    imu_factor->setVariableId(3, curr_vel_var->graphId());
    imu_factor->setMeasurement(imu_integrator);

    VarPoseGpsType* curr_gps_pose_var = new VarPoseGpsType();
    curr_gps_pose_var->setEstimate(curr_gps_pose);
    curr_gps_pose_var->setGraphId(graph_id++);
    graph->addVariable(VariableBasePtr(curr_gps_pose_var));

    FactorGpsType* gps_factor = new FactorGpsType();
    gps_factor->setVariableId(0, graph_id - 1);
    gps_factor->setMeasurement(curr_gps_pose);

    graph->addFactor(FactorBasePtr(imu_factor));
    graph->addFactor(FactorBasePtr(gps_factor));
    imu_integrator.reset();

    solver.setGraph(graph);
    solver.compute();

    std::cerr << solver.iterationStats() << std::endl;

    // solver compute
  }
}

void loadKittiData(KittiCalibration& kitti_calibration,
                   vector<ImuMeasurement>& imu_measurements,
                   vector<GpsMeasurement>& gps_measurements) {
  string line;

  // Read IMU metadata and compute relative sensor pose transforms
  // BodyPtx BodyPty BodyPtz BodyPrx BodyPry BodyPrz AccelerometerSigma GyroscopeSigma
  // IntegrationSigma AccelerometerBiasSigma GyroscopeBiasSigma AverageDelta
  ifstream imu_metadata("./data/KittiImuBiasedMetadata.txt");
  printf("-- Reading sensor metadata\n");

  getline(imu_metadata, line, '\n'); // ignore the first line
  // Load Kitti calibration
  getline(imu_metadata, line, '\n');
  sscanf(line.c_str(),
         "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
         &kitti_calibration.body_ptx,
         &kitti_calibration.body_pty,
         &kitti_calibration.body_ptz,
         &kitti_calibration.body_prx,
         &kitti_calibration.body_pry,
         &kitti_calibration.body_prz,
         &kitti_calibration.accelerometer_sigma,
         &kitti_calibration.gyroscope_sigma,
         &kitti_calibration.integration_sigma,
         &kitti_calibration.accelerometer_bias_sigma,
         &kitti_calibration.gyroscope_bias_sigma,
         &kitti_calibration.average_delta_t);
  printf("IMU metadata: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
         kitti_calibration.body_ptx,
         kitti_calibration.body_pty,
         kitti_calibration.body_ptz,
         kitti_calibration.body_prx,
         kitti_calibration.body_pry,
         kitti_calibration.body_prz,
         kitti_calibration.accelerometer_sigma,
         kitti_calibration.gyroscope_sigma,
         kitti_calibration.integration_sigma,
         kitti_calibration.accelerometer_bias_sigma,
         kitti_calibration.gyroscope_bias_sigma,
         kitti_calibration.average_delta_t);

  // Read IMU data
  // Time dt accelX accelY accelZ omegaX omegaY omegaZ
  printf("-- Reading IMU measurements from file\n");
  {
    ifstream imu_data("data/KittiImuBiased.txt");
    getline(imu_data, line, '\n'); // ignore the first line

    double time = 0, dt = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;
    while (!imu_data.eof()) {
      getline(imu_data, line, '\n');
      sscanf(line.c_str(),
             "%lf %lf %lf %lf %lf %lf %lf %lf",
             &time,
             &dt,
             &acc_x,
             &acc_y,
             &acc_z,
             &gyro_x,
             &gyro_y,
             &gyro_z);

      ImuMeasurement measurement;
      measurement.time          = time;
      measurement.dt            = dt;
      measurement.accelerometer = Vector3f(acc_x, acc_y, acc_z);
      measurement.gyroscope     = Vector3f(gyro_x, gyro_y, gyro_z);
      imu_measurements.push_back(measurement);
    }
  }

  // Read GPS data
  // Time,X,Y,Z
  printf("-- Reading GPS measurements from file\n");
  {
    ifstream gps_data("data/KittiGps.txt");
    getline(gps_data, line, '\n'); // ignore the first line

    double time = 0, gps_x = 0, gps_y = 0, gps_z = 0;
    while (!gps_data.eof()) {
      getline(gps_data, line, '\n');
      sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &time, &gps_x, &gps_y, &gps_z);

      GpsMeasurement measurement;
      measurement.time     = time;
      measurement.position = Vector3f(gps_x, gps_y, gps_z);
      gps_measurements.push_back(measurement);
    }
  }
}