#include <interfaces/pinocchio.h>


PinocchioSolver::PinocchioSolver() : model_(), data_(model_) {}

PinocchioSolver::PinocchioSolver(const std::string& urdfPath, const std::string& baseFrame, const std::string& endEffectorFrame)
  : baseFrame_(baseFrame), endEffectorFrame_(endEffectorFrame) {
  if (!loadFromURDF(urdfPath)) {
    throw std::runtime_error("Failed to load URDF!");
  }
}

bool PinocchioSolver::loadFromURDF(const std::string& urdfPath) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    pinocchio::urdf::buildModel(urdfPath, model_);
    data_ = pinocchio::Data(model_);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error loading URDF: " << e.what() << std::endl;
    return false;
  }
}

bool PinocchioSolver::isStateValid(const Eigen::VectorXd& jointAngles) const {
  // Add custom joint limits or validity checks
  return jointAngles.size() == this->getDOF();
}

int PinocchioSolver::getDOF() const {
    return model_.nq; // Number of degrees of freedom
}

Eigen::VectorXd PinocchioSolver::forwardKinematics(const Eigen::VectorXd& jointAngles) const {
  if (jointAngles.size() != this->getDOF()) {
    std::cerr << "Error: Joint angles size does not match model DOF" << std::endl;
    return Eigen::VectorXd();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pinocchio::forwardKinematics(model_, data_, jointAngles);

  Eigen::VectorXd pose(6); // Position (xyz) + Orientation (rpy)
  const pinocchio::SE3& endEffectorTransform = data_.oMf[model_.getFrameId(endEffectorFrame_)];

  pose.head<3>() = endEffectorTransform.translation();
  pose.tail<3>() = endEffectorTransform.rotation().eulerAngles(0, 1, 2); // XYZ Euler angles

  return pose;
}

Eigen::VectorXd PinocchioSolver::inverseKinematics(const Eigen::VectorXd& endEffectorPose) const {
  // TODO: IK implementation here (e.g., numerical, analytical)
  Eigen::VectorXd jointAngles = Eigen::VectorXd::Zero(model_.nq);
  return jointAngles;
}
