/**
 * @file   individual.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Fri Sep 16 15:09:56 2011
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
*
 * @brief
 *
 *
 */

#ifndef _INDIVIDUAL_HPP_
#define _INDIVIDUAL_HPP_

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/smart_ptr.hpp>

#include <sr_utilities/mtrand.h>

namespace shadow_robot
{
  template <class GeneType>
  class Individual
  {
  public:
    Individual(std::vector<GeneType> starting_seed, double max_mutation_percentage_rate = 0.5);
    virtual ~Individual();

    /**
     * Randomly mutates a gene of this individual.
     */
    void mutate();

    /**
     * Returns the fitness for this individual.
     * @return fitness value
     */
    double get_fitness()
    {
      return fitness;
    };


  protected:
    double fitness;

    ///random number generator
    sr_utilities::MTRand drand;
    boost::shared_ptr<sr_utilities::MTRangedRand<unsigned int> > range_rand;

    double max_mutation_percentage_rate;

    boost::ptr_vector<GeneType> genome;

    static const GeneType gene_max_percentage_change;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif