/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * Author: Stuart Glaser
 *
 * modified by Ugo Cupcic
 */
#ifndef _SR_SIMPLE_TRANSMISSION_H_
#define _SR_SIMPLE_TRANSMISSION_H_

#include <tinyxml.h>
#include "ros_ethercat_mechanism_model/transmission.hpp"
#include "ros_ethercat_mechanism_model/joint.hpp"
#include "ros_ethercat_hardware_interface/hardware_interface.hpp"
#include "ros_ethercat_mechanism_model/joint_calibration_simulator.hpp"

namespace sr_mechanism_model {

        class SimpleTransmissionForMuscle : public ros_ethercat_mechanism_model::Transmission
        {
        public:
                SimpleTransmissionForMuscle() {}
                ~SimpleTransmissionForMuscle() {}

                bool initXml(TiXmlElement *config, ros_ethercat_mechanism_model::Robot *robot);
                bool initXml(TiXmlElement *config);

                double mechanical_reduction_;

                void propagatePosition(std::vector<ros_ethercat_hardware_interface::Actuator*>&,
                                       std::vector<ros_ethercat_mechanism_model::JointState*>&);
                void propagatePositionBackwards(std::vector<ros_ethercat_mechanism_model::JointState*>&,
                                                std::vector<ros_ethercat_hardware_interface::Actuator*>&);
                void propagateEffort(std::vector<ros_ethercat_mechanism_model::JointState*>&,
                                     std::vector<ros_ethercat_hardware_interface::Actuator*>&);
                void propagateEffortBackwards(std::vector<ros_ethercat_hardware_interface::Actuator*>&,
                                              std::vector<ros_ethercat_mechanism_model::JointState*>&);

        private:
                int simulated_actuator_timestamp_initialized_;
                ros::Time simulated_actuator_start_time_;

                ros_ethercat_mechanism_model::JointCalibrationSimulator joint_calibration_simulator_;
        };

} // namespace ros_ethercat_mechanism_model

#endif
