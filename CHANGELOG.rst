^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grasp_synergy_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Discover grasp and knot names from profile parameters and sort knots by coordinate.
  Migration: remove ``grasp_names`` and ``knot_names`` from existing YAML files;
  all profiles under ``grasps`` are now loaded.
* Add configurable grasp profiles with piecewise-linear synergy knots.
* Expose standard JointTrajectory and FollowJointTrajectory endpoints per grasp.
* Forward commands to a remappable downstream JointTrajectoryController.
* Relay normalized action feedback, results, and cancellation.
* Contributors: Sunghyun Kwon
