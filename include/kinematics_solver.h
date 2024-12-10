#ifndef KINEMATICS_SOLVER_H
#define KINEMATICS_SOLVER_H

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <mutex>

class KinematicsSolver {
public:
  virtual ~KinematicsSolver() = default;
  virtual bool isStateValid(const Eigen::VectorXd& jointAngles) const = 0;
  virtual Eigen::VectorXd forwardKinematics(const Eigen::VectorXd& jointAngles) const = 0;
  virtual Eigen::VectorXd inverseKinematics(const Eigen::VectorXd& endEffectorPose) const = 0;
};

#endif // KINEMATICS_SOLVER_H
