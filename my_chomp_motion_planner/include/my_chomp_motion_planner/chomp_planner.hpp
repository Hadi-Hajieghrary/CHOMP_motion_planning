#pragma once

#include <my_chomp_motion_planner/chomp_parameters.hpp>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

namespace chomp
{
/**
 * \class ChompPlanner
 * \brief Provides a CHOMP-based planning interface compatible with MoveIt's planning pipeline.
 *
 * The \a solve() method creates an initial trajectory, calls the CHOMP optimizer,
 * and fills a MotionPlanDetailedResponse.
 */
class ChompPlanner
{
public:
  /**
   * \brief Default constructor
   */
  ChompPlanner() = default;

  /**
   * \brief Virtual destructor
   */
  virtual ~ChompPlanner() = default;

  /**
   * \brief Main entry point for CHOMP-based motion planning
   *
   * This function:
   *   1) Validates the \a planning_scene and \a req
   *   2) Builds an initial ChompTrajectory from start and goal states
   *   3) Initializes the trajectory (linear, cubic, quintic-spline, or from an input trajectory)
   *   4) Creates a ChompOptimizer, and runs optimization
   *   5) Fills a MotionPlanDetailedResponse with the optimized path
   *
   * \param[in]  planning_scene  Shared pointer to the planning scene (collision environment, transforms, etc.)
   * \param[in]  req            The motion planning request (includes start state, goal constraints, etc.)
   * \param[in]  params         ChompParameters controlling the optimization process
   * \param[out] res            The motion plan response (trajectory, result code, etc.)
   * \return true if a collision-free path was found, false otherwise
   */
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req,
             const ChompParameters& params,
             planning_interface::MotionPlanDetailedResponse& res) const;
};

}  // namespace chomp
