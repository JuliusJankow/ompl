/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_ANYTIMEOPTIMIZATION_ANYTIMEPATHSHORTENING_
#define OMPL_GEOMETRIC_PLANNERS_ANYTIMEOPTIMIZATION_ANYTIMEPATHSHORTENING_

#include "ompl/base/Planner.h"
#include <vector>
#include <thread>

namespace ompl
{
    namespace geometric
    {
        /// @anchor gAPS
        /// @par Short description
        /// Anytime path shortening is a generic wrapper around one or more
        /// geometric motion planners that repeatedly applies shortcutting
        /// (ompl::geometric::PathSimplifier) and hybridization
        /// (ompl::geometric::PathHybridization) to a set of solution paths
        /// with the goal of rapidly converging to a solution with
        /// minimal length.  Any number and combination of
        /// planners can be specified, each is run in a separate thread.
        /// A path length objective may be set, otherwise the tool will run
        /// until the termination condition is met.
        /// The purpose of this tool is to add anytime properties to motion
        /// planners that are not typically viewed as optimal/optimizing
        /// algorithms.
        ///
        /// @par External documentation
        /// R. Luna, I.A. Şucan, M. Moll, and L.E. Kavraki, Anytime Solution Optimization for Sampling-Based Motion
        /// Planning, in <em>Proc. 2013 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 5053-5059, May. 2013. DOI:
        /// [ICRA.2013.6631301](http://dx.doi.org/10.1109/ICRA.2013.6631301)<br>
        /// [[PDF]](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6631301)
        class AnytimePathShortening : public base::Planner
        {
        public:
            /// \brief Factory for creating a shared pointer to an AnytimePathShortening
            /// instance with numPlanners instances of planners of type PlannerType.
            template<typename PlannerType>
            static std::shared_ptr<AnytimePathShortening> createPlanner(
                const base::SpaceInformationPtr &si,
                unsigned int numPlanners = std::max(1u, std::thread::hardware_concurrency()))
            {
                auto result = std::make_shared<AnytimePathShortening>(si);
                result->planners_.reserve(numPlanners);
                for (unsigned int i = 0; i < numPlanners; ++i)
                    result->planners_.emplace_back(std::make_shared<PlannerType>(si));
                return result;
            }

            /// \brief Constructor requires the space information to plan in
            AnytimePathShortening(const base::SpaceInformationPtr &si);

            /// \brief Destructor
            ~AnytimePathShortening() override;

            /// \brief Adds the given planner to the set of planners used to
            /// compute candidate paths.
            void addPlanner(base::PlannerPtr &planner);

            /// \brief Set the problem definition for the planners. The
            /// problem needs to be set before calling solve(). Note:
            /// If this problem definition replaces a previous one, it
            /// may also be necessary to call clear().
            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /// \brief Method that solves the motion planning problem.  This method
            /// terminates under just two conditions, the given argument condition,
            /// or when the maximum path length in the optimization objective is met.
            /// \remarks This method spawns a separate thread for each planner
            /// employed, and applies a series of optimization methods to the set
            /// of paths generated by each planning thread.
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /// \brief Clear all internal planning datastructures. Planner
            /// settings are not affected. Subsequent calls to solve()
            /// will ignore all previous work.
            void clear() override;

            /// \brief Get information about the most recent run of the motion planner.
            /// \remarks This call is ambiguous in this tool.  By default, the
            /// planner data for the first planner in the planner list is returned.
            void getPlannerData(base::PlannerData &data) const override;

            /// \brief Get information about the most recent run of the idxth motion planner.
            virtual void getPlannerData(ompl::base::PlannerData &data, unsigned int idx) const;

            /// \brief Perform any necessary configuration steps.  This method
            /// also invokes ompl::base::SpaceInformation::setup() if needed. This
            /// must be called before solving
            void setup() override;

            /// \brief Check to see if the planners are in a working
            /// state (setup has been called, a goal was set, the
            /// input states seem to be in order). In case of error,
            /// this function throws an exception.
            void checkValidity() override;

            /// \brief Retrieve the number of planners added
            unsigned int getNumPlanners() const;

            /// \brief Retrieve a pointer to the ith planner instance
            base::PlannerPtr getPlanner(unsigned int idx) const;

            /// \brief Return whether the anytime planner will perform shortcutting on paths
            bool isShortcutting() const;

            /// \brief Enable/disable shortcutting on paths
            void setShortcut(bool shortcut);

            /// \brief Return whether the anytime planner will extract a hybrid path from the set of solution paths
            bool isHybridizing() const;

            /// \brief Enable/disable path hybridization on the set of solution paths
            void setHybridize(bool hybridize);

            /// \brief Return the maximum number of paths that will be hybridized
            unsigned int maxHybridizationPaths() const;

            /// \brief Set the maximum number of paths that will be hybridized
            void setMaxHybridizationPath(unsigned int maxPathCount);

            /// \brief Set default number of planners to use if none are specified.
            void setDefaultNumPlanners(unsigned int numPlanners);

            /// \brief Get default number of planners used if none are specified.
            unsigned int getDefaultNumPlanners() const;

            /** \brief Return best cost found so far by algorithm */
            std::string getBestCost() const;

        protected:
            /// \brief The function that the planning threads execute when
            /// solving a motion planning problem.
            virtual void threadSolve(base::Planner *planner, const base::PlannerTerminationCondition &ptc);

            /// \brief The list of planners used for solving the problem.
            std::vector<base::PlannerPtr> planners_;

            /// \brief Flag indicating whether to shortcut paths
            bool shortcut_{true};

            /// \brief Flag indicating whether to hybridize the set of solution paths
            bool hybridize_{true};

            /// \brief The maximum number of paths that will be hybridized.  This
            /// prohibits hybridization of a very large path set, which may take significant time.
            unsigned int maxHybridPaths_{24};

            /// \brief The number of planners to use if none are specified. This defaults to the number of cores.
            /// This parameter has no effect if planners have already been added.
            unsigned int defaultNumPlanners_;
        };
    }
}
#endif
