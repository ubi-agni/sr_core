/**
 * @file   sr_friction_compensation.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Aug 17 11:14:31 2011
 *
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  Compensate the tendon friction by adding a given value depending on the sign of the force demand.
 *
 *
 */

#ifndef _SR_FRICTION_COMPENSATION_HPP_
#define _SR_FRICTION_COMPENSATION_HPP_

#include <ros/node_handle.h>

#include <boost/scoped_ptr.hpp>

#include <utility>


#include <control_toolbox/pid.h>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>


#include <sr_robot_msgs/SetMixedPositionVelocityPidGains.h>

#include <sr_utilities/calibration.hpp>

namespace sr_friction_compensation
{

  class SrFrictionCompensator
  {
  public:

    SrFrictionCompensator(const std::string &joint_name);

    ~SrFrictionCompensator();

    /**
     * Computes the value of the offset to add to the force demand
     * to compensate for the friction. This offset is computed based
     * on the current joint position and on the force demand sign as
     * well, using a friction map.
     *
     * The friction map is generated by moving the joint very slowly,
     * from one end to the other, while recording the force necessary
     * to move it. Two different map position->force are then created
     * by interpolating those values, one for each direction.
     *
     * @param position the current joint position.
     * @param velocity the current joint velocity.
     * @param force_demand the force demand is used to know in which direction
     *                     we want to move:
     *                          if > 0 -> forward
     *                          if < 0 -> backward
     * @param deadband the deadband on the force_demand (in the deadband region,
     *                 we're returning an offset of 0.0 for stability reasons).
     *
     * @return the force necessary to have the joint ready to move.
     */
    double friction_compensation(double position, double velocity, int force_demand, int deadband);

  private:
    ///node handle for reading the map from the parameters server.
    ros::NodeHandle node_;

    /**
     * Read the 2 friction maps from the parameter servers for each joint.
     * Generate a flat map (no offset) if the map doesn't exist.
     *
     *
     * @return a pair of map: the first is the forward map, the second
     *         is the backward map.
     */
    std::pair<std::vector<joint_calibration::Point>, std::vector<joint_calibration::Point> > read_friction_map();

    /**
     * Format one map from the given raw map directly taken from the parameter
     * server.
     *
     * @param raw_map the raw_map directly read from the parameters.
     *
     * @return The map (either forward or backward)
     */
    std::vector<joint_calibration::Point> read_one_way_map(XmlRpc::XmlRpcValue &raw_map);

    /**
     * Generates a flat map for the joints missing one of their friction map.
     *
     *
     * @return A flat map (always 0.0)
     */
    std::vector<joint_calibration::Point> generate_flat_map();

    /// An interpolator for the forward friction map: used to compute the offset from the map, given the current position.
    boost::scoped_ptr<shadow_robot::JointCalibration> friction_interpoler_forward;
    /// An interpolator for the backward friction map: used to compute the offset from the map, given the current position.
    boost::scoped_ptr<shadow_robot::JointCalibration> friction_interpoler_backward;

    ///the joint name
    std::string joint_name_;

    ///the threshold under which we use the static friction map
    static const double velocity_for_static_friction;
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
