//Copyright  (C)  2020  Ruben Smits <ruben dot smits at intermodalics dot eu>
//
//Version: 1.0
//Author: Ruben Smits Ruben Smits <ruben dot smits at intermodalics dot eu>
//Author: Zihan Chen <zihan dot chen dot jhu at gmail dot com>
//Author: Matthijs van der Burgh <MatthijsBurgh at outlook dot com>
//Maintainer: Ruben Smits Ruben Smits <ruben dot smits at intermodalics dot eu>
//URL: http://www.orocos.org/kdl
//
//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public
//License as published by the Free Software Foundation; either
//version 2.1 of the License, or (at your option) any later version.
//
//This library is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//Lesser General Public License for more details.
//
//You should have received a copy of the GNU Lesser General Public
//License along with this library; if not, write to the Free Software
//Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit/py_bindings_tools/gil_releaser.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlannerInterfaceDescription.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <moveit_msgs/PickupGoal.h>
#include <moveit_msgs/PlaceGoal.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveGroupAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/Plane.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <tf2_ros/buffer.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

namespace py = pybind11;

class MoveGroupInterfaceWrapper : public moveit::py_bindings_tools::ROScppInitializer, public moveit::planning_interface::MoveGroupInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if
  // needed
  MoveGroupInterfaceWrapper(const std::string& group_name, const std::string& robot_description,
                            const std::string& ns = "", double wait_for_servers = 5.0)
    : moveit::py_bindings_tools::ROScppInitializer()
    , moveit::planning_interface::MoveGroupInterface(moveit::planning_interface::MoveGroupInterface::Options(group_name, robot_description, ros::NodeHandle(ns)),
                         std::shared_ptr<tf2_ros::Buffer>(), ros::WallDuration(wait_for_servers))
  {
  }

  moveit::planning_interface::MoveGroupInterface::Plan planWrapper()
  {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = this->plan(my_plan);
    return my_plan;
  }

};

class PlanningSceneInterfaceWrapper : public moveit::py_bindings_tools::ROScppInitializer, public moveit::planning_interface::PlanningSceneInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  PlanningSceneInterfaceWrapper(const std::string& ns = "")
    : moveit::py_bindings_tools::ROScppInitializer(), moveit::planning_interface::PlanningSceneInterface(ns)
  {
  }
};


PYBIND11_MODULE(moveit_pybind, m) {
  // --------------
  // Geometric
  // Pose
  py::class_<geometry_msgs::Pose> pose(m, "Pose");
  pose.def(py::init<>());
  pose.def_readwrite("position", &geometry_msgs::Pose::position);
  pose.def_readwrite("orientation", &geometry_msgs::Pose::orientation);

  // Point
  py::class_<geometry_msgs::Point> point(m, "Point");
  point.def(py::init<>());
  point.def_readwrite("x", &geometry_msgs::Point::x);
  point.def_readwrite("y", &geometry_msgs::Point::y);
  point.def_readwrite("z", &geometry_msgs::Point::z);

  // Quaternion
  py::class_<geometry_msgs::Quaternion> quat(m, "Quaternion");
  quat.def(py::init<>());
  quat.def_readwrite("x", &geometry_msgs::Quaternion::x);
  quat.def_readwrite("y", &geometry_msgs::Quaternion::y);
  quat.def_readwrite("z", &geometry_msgs::Quaternion::z);
  quat.def_readwrite("w", &geometry_msgs::Quaternion::w);

  // --------------
  // Moveit msgs
  // RobotTrajectory
  py::class_<moveit_msgs::RobotTrajectory> robot_traj(m, "RobotTrajectory");
  robot_traj.def(py::init<>());
  robot_traj.def_readwrite("joint_trajectory", &moveit_msgs::RobotTrajectory::joint_trajectory);
  robot_traj.def_readwrite("multi_dof_joint_trajectory", &moveit_msgs::RobotTrajectory::multi_dof_joint_trajectory);

  // RobotState
  py::class_<moveit_msgs::RobotState> robot_state(m, "RobotState");
  robot_state.def(py::init<>());
  robot_state.def_readwrite("joint_state", &moveit_msgs::RobotState::joint_state);
  robot_state.def_readwrite("multi_dof_joint_state", &moveit_msgs::RobotState::multi_dof_joint_state);
  robot_state.def_readwrite("attached_collision_objects", &moveit_msgs::RobotState::attached_collision_objects);
  robot_state.def_readwrite("is_diff", &moveit_msgs::RobotState::is_diff);

  // --------------
  // Trajectory msgs
  // JointTrajectory
  py::class_<trajectory_msgs::JointTrajectory> joint_traj(m, "JointTrajectory");
  joint_traj.def(py::init<>());
  joint_traj.def_readwrite("header", &trajectory_msgs::JointTrajectory::header);
  joint_traj.def_readwrite("joint_names", &trajectory_msgs::JointTrajectory::joint_names);
  joint_traj.def_readwrite("points", &trajectory_msgs::JointTrajectory::points);

  py::class_<trajectory_msgs::JointTrajectoryPoint> joint_traj_pt(m, "JointTrajectoryPoint");
  joint_traj_pt.def(py::init<>());
  joint_traj_pt.def_readwrite("positions", &trajectory_msgs::JointTrajectoryPoint::positions);
  joint_traj_pt.def_readwrite("velocities", &trajectory_msgs::JointTrajectoryPoint::velocities);
  joint_traj_pt.def_readwrite("accelerations", &trajectory_msgs::JointTrajectoryPoint::accelerations);
  joint_traj_pt.def_readwrite("effort", &trajectory_msgs::JointTrajectoryPoint::effort);
  joint_traj_pt.def_readwrite("time_from_start", &trajectory_msgs::JointTrajectoryPoint::time_from_start);

  // MultiDOFJointTrajectory
  py::class_<trajectory_msgs::MultiDOFJointTrajectory> multi_dof_joint_traj(m, "MultiDOFJointTrajectory");
  multi_dof_joint_traj.def(py::init<>());
  multi_dof_joint_traj.def_readwrite("header", &trajectory_msgs::MultiDOFJointTrajectory::header);
  multi_dof_joint_traj.def_readwrite("joint_names", &trajectory_msgs::MultiDOFJointTrajectory::joint_names);
  multi_dof_joint_traj.def_readwrite("points", &trajectory_msgs::MultiDOFJointTrajectory::points);

  py::class_<trajectory_msgs::MultiDOFJointTrajectoryPoint> multi_dof_joint_traj_pt(m, "MultiDOFJointTrajectoryPoint");
  multi_dof_joint_traj_pt.def(py::init<>());
  multi_dof_joint_traj_pt.def_readwrite("transforms", &trajectory_msgs::MultiDOFJointTrajectoryPoint::transforms);
  multi_dof_joint_traj_pt.def_readwrite("velocities", &trajectory_msgs::MultiDOFJointTrajectoryPoint::velocities);
  multi_dof_joint_traj_pt.def_readwrite("accelerations", &trajectory_msgs::MultiDOFJointTrajectoryPoint::accelerations);
  multi_dof_joint_traj_pt.def_readwrite("time_from_start", &trajectory_msgs::MultiDOFJointTrajectoryPoint::time_from_start);


  // --------------
  // Plan
  py::class_<moveit::planning_interface::MoveGroupInterface::Plan> plan(m, "Plan");
  plan.def(py::init<moveit_msgs::RobotState, moveit_msgs::RobotTrajectory, double>());
  plan.def(py::init<>());
  plan.def_readwrite("start_state", &moveit::planning_interface::MoveGroupInterface::Plan::start_state_);
  plan.def_readwrite("trajectory", &moveit::planning_interface::MoveGroupInterface::Plan::trajectory_);
  plan.def_readwrite("planning_time", &moveit::planning_interface::MoveGroupInterface::Plan::planning_time_);

  // --------------
  // MoveItErrorCode
  py::class_<moveit::planning_interface::MoveItErrorCode> ec(m, "MoveItErrorCode");
  ec.def(py::init<>());
  ec.def_readwrite("val", &moveit::planning_interface::MoveItErrorCode::val);
  
  // --------------
  // MoveGroupInterface
  py::class_<MoveGroupInterfaceWrapper> mgi(m, "MoveGroupInterfaceWrapper");
  mgi.def(py::init<const std::string&, const std::string&, const std::string&, double>(),
    py::arg("group_name"), py::arg("robot_description"), py::arg("ns")="", py::arg("wait_for_servers")=5.0);

  mgi.def("getName", &MoveGroupInterfaceWrapper::getName);
  mgi.def("getNamedTargets", &MoveGroupInterfaceWrapper::getNamedTargets);
  mgi.def("getPlanningFrame", &MoveGroupInterfaceWrapper::getPlanningFrame);
  mgi.def("getJointModelGroupNames", &MoveGroupInterfaceWrapper::getJointModelGroupNames);
  mgi.def("getJointNames", &MoveGroupInterfaceWrapper::getJointNames);
  mgi.def("getLinkNames", &MoveGroupInterfaceWrapper::getLinkNames);
  mgi.def("getNamedTargetValues", &MoveGroupInterfaceWrapper::getNamedTargetValues);
  mgi.def("getActiveJoints", &MoveGroupInterfaceWrapper::getActiveJoints);
  mgi.def("getJoints", &MoveGroupInterfaceWrapper::getJoints);
  mgi.def("setPlannerParams", &MoveGroupInterfaceWrapper::setPlannerParams);
  mgi.def("getDefaultPlannerId", &MoveGroupInterfaceWrapper::getDefaultPlannerId);
  mgi.def("setPlannerId", &MoveGroupInterfaceWrapper::setPlannerId);
  mgi.def("getPlannerId", &MoveGroupInterfaceWrapper::getPlannerId);
  mgi.def("setPlanningTime", &MoveGroupInterfaceWrapper::setPlanningTime);
  mgi.def("setNumPlanningAttempts", &MoveGroupInterfaceWrapper::setNumPlanningAttempts);
  mgi.def("setMaxVelocityScalingFactor", &MoveGroupInterfaceWrapper::setMaxVelocityScalingFactor);

  mgi.def("getPlanningTime", &MoveGroupInterfaceWrapper::getPlanningTime);
  mgi.def("getGoalJointTolerance", &MoveGroupInterfaceWrapper::getGoalJointTolerance);
  mgi.def("getGoalPositionTolerance", &MoveGroupInterfaceWrapper::getGoalPositionTolerance);
  mgi.def("getGoalOrientationTolerance", &MoveGroupInterfaceWrapper::getGoalOrientationTolerance);
  mgi.def("setGoalTolerance", &MoveGroupInterfaceWrapper::setGoalTolerance);
  mgi.def("setGoalJointTolerance", &MoveGroupInterfaceWrapper::setGoalJointTolerance);
  mgi.def("setGoalPositionTolerance", &MoveGroupInterfaceWrapper::setGoalPositionTolerance);
  mgi.def("setGoalOrientationTolerance", &MoveGroupInterfaceWrapper::setGoalOrientationTolerance);
  mgi.def("setWorkspace", &MoveGroupInterfaceWrapper::setWorkspace);

  mgi.def("setStartState", (void (MoveGroupInterfaceWrapper::*)(const moveit_msgs::RobotState&)) &MoveGroupInterfaceWrapper::setStartState);
  mgi.def("setStartState", (void (MoveGroupInterfaceWrapper::*)(const robot_state::RobotState&)) &MoveGroupInterfaceWrapper::setStartState);

  mgi.def("setStartStateToCurrentState", &MoveGroupInterfaceWrapper::setStartStateToCurrentState);
  mgi.def("setSupportSurfaceName", &MoveGroupInterfaceWrapper::setSupportSurfaceName);

  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const std::vector<double>&)) &MoveGroupInterfaceWrapper::setJointValueTarget);
  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const std::map<std::string, double>&)) &MoveGroupInterfaceWrapper::setJointValueTarget);
  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const robot_state::RobotState&)) &MoveGroupInterfaceWrapper::setJointValueTarget);
  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const std::string&, const std::vector<double>&)) &MoveGroupInterfaceWrapper::setJointValueTarget);
  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const std::string&, double)) &MoveGroupInterfaceWrapper::setJointValueTarget);
  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const sensor_msgs::JointState&)) &MoveGroupInterfaceWrapper::setJointValueTarget);
  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const geometry_msgs::Pose&, const std::string&)) &MoveGroupInterfaceWrapper::setJointValueTarget, 
    py::arg("eef_pose"), py::arg("end_effector_link")="");
  mgi.def("setJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const geometry_msgs::PoseStamped&, const std::string&)) &MoveGroupInterfaceWrapper::setJointValueTarget, 
    py::arg("eef_pose"), py::arg("end_effector_link")="");

  mgi.def("setApproximateJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const geometry_msgs::Pose&, const std::string&)) &MoveGroupInterfaceWrapper::setApproximateJointValueTarget, 
    py::arg("eef_pose"), py::arg("end_effector_link")="");
  mgi.def("setApproximateJointValueTarget", (bool (MoveGroupInterfaceWrapper::*)(const geometry_msgs::PoseStamped&, const std::string&)) &MoveGroupInterfaceWrapper::setApproximateJointValueTarget, 
    py::arg("eef_pose"), py::arg("end_effector_link")="");

  mgi.def("setRandomTarget", &MoveGroupInterfaceWrapper::setRandomTarget);
  mgi.def("setNamedTarget", &MoveGroupInterfaceWrapper::setNamedTarget);

  mgi.def("getJointValueTarget", &MoveGroupInterfaceWrapper::getJointValueTarget);
  mgi.def("setPositionTarget", &MoveGroupInterfaceWrapper::setPositionTarget);
  mgi.def("setRPYTarget", &MoveGroupInterfaceWrapper::setRPYTarget);
  mgi.def("setOrientationTarget", &MoveGroupInterfaceWrapper::setOrientationTarget);

  mgi.def("setPoseTarget", (bool (MoveGroupInterfaceWrapper::*)(const geometry_msgs::Pose&, const std::string&)) &MoveGroupInterfaceWrapper::setPoseTarget, 
    py::arg("target"), py::arg("end_effector_link")="");
  mgi.def("setPoseTarget", (bool (MoveGroupInterfaceWrapper::*)(const geometry_msgs::PoseStamped&, const std::string&)) &MoveGroupInterfaceWrapper::setPoseTarget, 
    py::arg("target"), py::arg("end_effector_link")="");

  mgi.def("setPoseTargets", (bool (MoveGroupInterfaceWrapper::*)(const std::vector<geometry_msgs::Pose>&, const std::string&)) &MoveGroupInterfaceWrapper::setPoseTargets, 
    py::arg("target"), py::arg("end_effector_link")="");
  mgi.def("setPoseTargets", (bool (MoveGroupInterfaceWrapper::*)(const std::vector<geometry_msgs::PoseStamped>&, const std::string&)) &MoveGroupInterfaceWrapper::setPoseTargets, 
    py::arg("target"), py::arg("end_effector_link")="");

  mgi.def("setPoseReferenceFrame", &MoveGroupInterfaceWrapper::setPoseReferenceFrame);
  mgi.def("setEndEffectorLink", &MoveGroupInterfaceWrapper::setEndEffectorLink);
  mgi.def("setEndEffector", &MoveGroupInterfaceWrapper::setEndEffector);
  mgi.def("clearPoseTarget", &MoveGroupInterfaceWrapper::clearPoseTarget, py::arg("end_effector_link")="");
  mgi.def("clearPoseTargets", &MoveGroupInterfaceWrapper::clearPoseTargets);
  mgi.def("getPoseTarget", &MoveGroupInterfaceWrapper::getPoseTarget, py::arg("end_effector_link")="");
  mgi.def("getPoseTargets", &MoveGroupInterfaceWrapper::getPoseTargets, py::arg("end_effector_link")="");
  mgi.def("getEndEffectorLink", &MoveGroupInterfaceWrapper::getEndEffectorLink);
  mgi.def("getEndEffector", &MoveGroupInterfaceWrapper::getEndEffector);
  mgi.def("getPoseReferenceFrame", &MoveGroupInterfaceWrapper::getPoseReferenceFrame);
  mgi.def("asyncMove", &MoveGroupInterfaceWrapper::asyncMove);
  mgi.def("getMoveGroupClient", &MoveGroupInterfaceWrapper::getMoveGroupClient);
  mgi.def("move", &MoveGroupInterfaceWrapper::move);
  mgi.def("plan", &MoveGroupInterfaceWrapper::planWrapper);

  mgi.def("asyncExecute", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const moveit::planning_interface::MoveGroupInterface::Plan&)) &MoveGroupInterfaceWrapper::asyncExecute);
  mgi.def("asyncExecute", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const moveit_msgs::RobotTrajectory&)) &MoveGroupInterfaceWrapper::asyncExecute);

  mgi.def("execute", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const moveit::planning_interface::MoveGroupInterface::Plan&)) &MoveGroupInterfaceWrapper::execute);
  mgi.def("execute", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const moveit_msgs::RobotTrajectory&)) &MoveGroupInterfaceWrapper::execute);

  mgi.def("computeCartesianPath", (double (MoveGroupInterfaceWrapper::*)(const std::vector<geometry_msgs::Pose>&, double, double, moveit_msgs::RobotTrajectory&, bool, moveit_msgs::MoveItErrorCodes*)) &MoveGroupInterfaceWrapper::computeCartesianPath, 
    py::arg("waypoints"), py::arg("eef_step"), py::arg("jump_threshold"), py::arg("trajectory"), py::arg("avoid_collisions")=true, py::arg("error_code")=NULL);
  mgi.def("computeCartesianPath", (double (MoveGroupInterfaceWrapper::*)(const std::vector<geometry_msgs::Pose>&, double, double, moveit_msgs::RobotTrajectory&, const moveit_msgs::Constraints&, bool, moveit_msgs::MoveItErrorCodes*)) &MoveGroupInterfaceWrapper::computeCartesianPath, 
    py::arg("waypoints"), py::arg("eef_step"), py::arg("jump_threshold"), py::arg("trajectory"), py::arg("path_constraints"), py::arg("avoid_collisions")=true, py::arg("error_code")=NULL);

  mgi.def("stop", &MoveGroupInterfaceWrapper::stop);
  mgi.def("allowLooking", &MoveGroupInterfaceWrapper::allowLooking);
  mgi.def("allowReplanning", &MoveGroupInterfaceWrapper::allowReplanning);
  mgi.def("constructMotionPlanRequest", &MoveGroupInterfaceWrapper::constructMotionPlanRequest);
  mgi.def("constructPickupGoal", &MoveGroupInterfaceWrapper::constructPickupGoal);
  mgi.def("constructPlaceGoal", &MoveGroupInterfaceWrapper::constructPlaceGoal);
  mgi.def("posesToPlaceLocations", &MoveGroupInterfaceWrapper::posesToPlaceLocations);

  mgi.def("pick", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, bool)) &MoveGroupInterfaceWrapper::pick, 
    py::arg("object"), py::arg("plan_only")=false);
  mgi.def("pick", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, const moveit_msgs::Grasp&, bool)) &MoveGroupInterfaceWrapper::pick, 
    py::arg("object"), py::arg("grasp"), py::arg("plan_only")=false);
  mgi.def("pick", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, std::vector<moveit_msgs::Grasp>, bool)) &MoveGroupInterfaceWrapper::pick, 
    py::arg("object"), py::arg("grasps"), py::arg("plan_only")=false);
  mgi.def("pick", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const moveit_msgs::PickupGoal&)) &MoveGroupInterfaceWrapper::pick, 
    py::arg("goal"));

  mgi.def("planGraspsAndPick", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, bool)) &MoveGroupInterfaceWrapper::planGraspsAndPick, 
    py::arg("object")="", py::arg("plan_only")=false);
  mgi.def("planGraspsAndPick", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const moveit_msgs::CollisionObject&, bool)) &MoveGroupInterfaceWrapper::planGraspsAndPick, 
    py::arg("object"), py::arg("plan_only")=false);

  mgi.def("place", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, bool)) &MoveGroupInterfaceWrapper::place, 
    py::arg("object"), py::arg("plan_only")=false);
  mgi.def("place", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, std::vector<moveit_msgs::PlaceLocation>, bool)) &MoveGroupInterfaceWrapper::place, 
    py::arg("object"), py::arg("locations"), py::arg("plan_only")=false);
  mgi.def("place", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, const std::vector<geometry_msgs::PoseStamped>&, bool)) &MoveGroupInterfaceWrapper::place, 
    py::arg("object"), py::arg("poses"), py::arg("plan_only")=false);
  mgi.def("place", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const std::string&, const geometry_msgs::PoseStamped&, bool)) &MoveGroupInterfaceWrapper::place, 
    py::arg("object"), py::arg("pose"), py::arg("plan_only")=false);
  mgi.def("place", (moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*)(const moveit_msgs::PlaceGoal&)) &MoveGroupInterfaceWrapper::place, 
    py::arg("goal"));

  mgi.def("attachObject", (bool (MoveGroupInterfaceWrapper::*)(const std::string&, const std::string&)) &MoveGroupInterfaceWrapper::attachObject, 
    py::arg("object"), py::arg("link")="");
  mgi.def("attachObject", (bool (MoveGroupInterfaceWrapper::*)(const std::string&, const std::string&, const std::vector<std::string>&)) &MoveGroupInterfaceWrapper::attachObject);

  mgi.def("detachObject", &MoveGroupInterfaceWrapper::detachObject, py::arg("name")="");
  mgi.def("startStateMonitor", &MoveGroupInterfaceWrapper::startStateMonitor, py::arg("wait")=1.0);
  mgi.def("getCurrentJointValues", &MoveGroupInterfaceWrapper::getCurrentJointValues);
  mgi.def("getCurrentState", &MoveGroupInterfaceWrapper::getCurrentState, py::arg("wait")=1);
  mgi.def("getCurrentPose", &MoveGroupInterfaceWrapper::getCurrentPose, py::arg("end_effector_link")="");
  mgi.def("getCurrentRPY", &MoveGroupInterfaceWrapper::getCurrentRPY, py::arg("end_effector_link")="");
  mgi.def("getRandomJointValues", &MoveGroupInterfaceWrapper::getRandomJointValues);
  mgi.def("getRandomPose", &MoveGroupInterfaceWrapper::getRandomPose, py::arg("end_effector_link")="");

  mgi.def("rememberJointValues", (void (MoveGroupInterfaceWrapper::*)(const std::string&)) &MoveGroupInterfaceWrapper::rememberJointValues);
  mgi.def("rememberJointValues", (void (MoveGroupInterfaceWrapper::*)(const std::string&, const std::vector<double>&)) &MoveGroupInterfaceWrapper::rememberJointValues);
  
  mgi.def("getRememberedJointValues", &MoveGroupInterfaceWrapper::getRememberedJointValues);
  mgi.def("forgetJointValues", &MoveGroupInterfaceWrapper::forgetJointValues);
  mgi.def("setConstraintsDatabase", &MoveGroupInterfaceWrapper::setConstraintsDatabase);
  mgi.def("getKnownConstraints", &MoveGroupInterfaceWrapper::getKnownConstraints);
  mgi.def("getPathConstraints", &MoveGroupInterfaceWrapper::getPathConstraints);

  mgi.def("setPathConstraints", (bool (MoveGroupInterfaceWrapper::*)(const std::string&)) &MoveGroupInterfaceWrapper::setPathConstraints);
  mgi.def("setPathConstraints", (void (MoveGroupInterfaceWrapper::*)(const moveit_msgs::Constraints&)) &MoveGroupInterfaceWrapper::setPathConstraints);
  
  mgi.def("clearPathConstraints", &MoveGroupInterfaceWrapper::clearPathConstraints);
  mgi.def("getTrajectoryConstraints", &MoveGroupInterfaceWrapper::getTrajectoryConstraints);
  mgi.def("setTrajectoryConstraints", &MoveGroupInterfaceWrapper::setTrajectoryConstraints);
  mgi.def("clearTrajectoryConstraints", &MoveGroupInterfaceWrapper::clearTrajectoryConstraints);

  // --------------
  // PlanningSceneInterface
  py::class_<PlanningSceneInterfaceWrapper> psi(m, "PlanningSceneInterfaceWrapper");
  psi.def(py::init<const std::string&>(), py::arg("ns")="");

  psi.def("getKnownObjectNames", &PlanningSceneInterfaceWrapper::getKnownObjectNames, py::arg("with_type")=false);

  psi.def("getKnownObjectNamesInROI", (std::vector<std::string> (PlanningSceneInterfaceWrapper::*)(
    double, double, double, double, double, double, bool, std::vector<std::string>&)) &PlanningSceneInterfaceWrapper::getKnownObjectNamesInROI
  );
  psi.def("getKnownObjectNamesInROI", (std::vector<std::string> (PlanningSceneInterfaceWrapper::*)(
    double, double, double, double, double, double, bool)) &PlanningSceneInterfaceWrapper::getKnownObjectNamesInROI, 
    py::arg("minx"), py::arg("miny"), py::arg("minz"), py::arg("maxx"), py::arg("maxy"), py::arg("maxz"), py::arg("with_type")=false
  );

  psi.def("getObjectPoses", &PlanningSceneInterfaceWrapper::getObjectPoses);
  psi.def("getObjects", &PlanningSceneInterfaceWrapper::getObjects, py::arg("object_ids")=std::vector<std::string>());
  psi.def("getAttachedObjects", &PlanningSceneInterfaceWrapper::getAttachedObjects, py::arg("object_ids")=std::vector<std::string>());

  psi.def("applyCollisionObject", (bool (PlanningSceneInterfaceWrapper::*)(const moveit_msgs::CollisionObject&)) &PlanningSceneInterfaceWrapper::applyCollisionObject);
  psi.def("applyCollisionObject", (bool (PlanningSceneInterfaceWrapper::*)(const moveit_msgs::CollisionObject&, const std_msgs::ColorRGBA&)) &PlanningSceneInterfaceWrapper::applyCollisionObject);

  psi.def("applyCollisionObjects", &PlanningSceneInterfaceWrapper::applyCollisionObjects, py::arg("collision_objects"), py::arg("object_colors")=std::vector<moveit_msgs::ObjectColor>());
  psi.def("applyAttachedCollisionObject", &PlanningSceneInterfaceWrapper::applyAttachedCollisionObject);
  psi.def("applyAttachedCollisionObjects", &PlanningSceneInterfaceWrapper::applyAttachedCollisionObjects);
  psi.def("applyPlanningScene", &PlanningSceneInterfaceWrapper::applyPlanningScene);
  psi.def("addCollisionObjects", &PlanningSceneInterfaceWrapper::addCollisionObjects, py::arg("collision_objects"), py::arg("object_colors")=std::vector<moveit_msgs::ObjectColor>());
  psi.def("removeCollisionObjects", &PlanningSceneInterfaceWrapper::removeCollisionObjects);

  // --------------
  // moveit_msgs
  // CollisionObject
  py::class_<moveit_msgs::CollisionObject> co(m, "CollisionObject");
  co.def(py::init<>());
  co.def_readwrite("header", &moveit_msgs::CollisionObject::header);
  co.def_readwrite("id", &moveit_msgs::CollisionObject::id);
  co.def_readwrite("type", &moveit_msgs::CollisionObject::type);
  co.def_readwrite("primitives", &moveit_msgs::CollisionObject::primitives);
  co.def_readwrite("primitive_poses", &moveit_msgs::CollisionObject::primitive_poses);
  co.def_readwrite("meshes", &moveit_msgs::CollisionObject::meshes);
  co.def_readwrite("mesh_poses", &moveit_msgs::CollisionObject::mesh_poses);
  co.def_readwrite("planes", &moveit_msgs::CollisionObject::planes);
  co.def_readwrite("plane_poses", &moveit_msgs::CollisionObject::plane_poses);
  co.def_readwrite("operation", &moveit_msgs::CollisionObject::operation);

  // --------------
  // shape_msgs
  // SolidPrimitive
  py::class_<shape_msgs::SolidPrimitive> sp(m, "SolidPrimitive");
  sp.def(py::init<>());
  sp.def_readwrite("type", &shape_msgs::SolidPrimitive::type);
  sp.def_readwrite("dimensions", &shape_msgs::SolidPrimitive::dimensions);

  // --------------
  // shape_msgs
  // Mesh
  py::class_<shape_msgs::Mesh> mesh(m, "Mesh");
  mesh.def(py::init<>());
  mesh.def_readwrite("triangles", &shape_msgs::Mesh::triangles);
  mesh.def_readwrite("vertices", &shape_msgs::Mesh::vertices);

  // --------------
  // shape_msgs
  // MeshTriangle
  py::class_<shape_msgs::MeshTriangle> mt(m, "MeshTriangle");
  mt.def(py::init<>());
  mt.def_readwrite("vertex_indices", &shape_msgs::MeshTriangle::vertex_indices);

  // --------------
  // shape_msgs
  // Plane
  py::class_<shape_msgs::Plane> plane(m, "Plane");
  plane.def(py::init<>());
  plane.def_readwrite("coef", &shape_msgs::Plane::coef);

  // --------------
  // moveit_msgs
  // AttachedCollisionObject
  py::class_<moveit_msgs::AttachedCollisionObject> aco(m, "AttachedCollisionObject");
  aco.def(py::init<>());
  aco.def_readwrite("link_name", &moveit_msgs::AttachedCollisionObject::link_name);
  aco.def_readwrite("object", &moveit_msgs::AttachedCollisionObject::object);
  aco.def_readwrite("touch_links", &moveit_msgs::AttachedCollisionObject::touch_links);
  aco.def_readwrite("detach_posture", &moveit_msgs::AttachedCollisionObject::detach_posture);
  aco.def_readwrite("weight", &moveit_msgs::AttachedCollisionObject::weight);
  
}
