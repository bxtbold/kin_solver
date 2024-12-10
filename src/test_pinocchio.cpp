#include <iostream>
#include <kinematics_solver.h>
#include <interfaces/pinocchio.h>

int main() {
  // Example usage
  std::cout << "Kinematic Solver Example" << std::endl;
  std::string urdfPath = "../model/simple_6dof.urdf"; // Specify your URDF file

  PinocchioSolver pinSolver;
  if (pinSolver.loadFromURDF(urdfPath)) {
    std::cout << "Pinocchio Solver loaded from URDF!" << std::endl;
    int dof = pinSolver.getDOF();
    Eigen::VectorXd test_angle = Eigen::VectorXd(dof);
    Eigen::VectorXd pose = pinSolver.forwardKinematics(test_angle);

    std::cout << "FK result " << std::endl;
    std::cout << pose << std::endl;
  } else {
    std::cerr << "Failed to load Pinocchio solver from URDF." << std::endl;
  }
  return 0;
}
