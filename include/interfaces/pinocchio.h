#ifndef PINOCCHIO_SOLVER_H
#define PINOCCHIO_SOLVER_H

#include <kinematics_solver.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>


class PinocchioSolver : public KinematicsSolver {
public:
  PinocchioSolver();
  PinocchioSolver(const std::string& urdfPath, const std::string& baseFrame, const std::string& endEffectorFrame);

  bool loadFromURDF(const std::string& urdfPath);
  bool isStateValid(const Eigen::VectorXd& jointAngles) const override;
  int getDOF() const;
  Eigen::VectorXd forwardKinematics(const Eigen::VectorXd& jointAngles) const override;
  Eigen::VectorXd inverseKinematics(const Eigen::VectorXd& endEffectorPose) const override;

private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  std::string baseFrame_;
  std::string endEffectorFrame_;
  mutable std::mutex mutex_;
};
#endif // PINOCCHIO_SOLVER_H
