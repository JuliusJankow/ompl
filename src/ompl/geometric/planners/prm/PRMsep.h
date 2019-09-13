#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_PRM_SEP_
#define OMPL_GEOMETRIC_PLANNERS_PRM_PRM_SEP_

#include "ompl/geometric/planners/prm/PRM.h"
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>

namespace ompl
{
    namespace geometric
    {
        /** \brief PRMsep planner */
        class PRMsep : public PRM
        {
        public:
            /** \brief Constructor */
            PRMsep(const base::SpaceInformationPtr &si);
            /** \brief Constructor */
            PRMsep(const base::SpaceInformationPtr &si, base::PlannerData* pd);

            void setPlannerData(base::PlannerData* pd);

            void setup() override; // always call setup after a new problem definition has been set

            void clear() override;

            void insertStartAndGoalStates(unsigned int min_number_goals);

            void constructRoadmap(const unsigned int max_number_milestones);

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            double debugFreeSpace(base::State *state);

            void resetPlannerData() {
                planner_data_set_ = false;
                setup_ = false;
            }

        protected:
            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
                This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(ompl::base::PlannerData::Graph::Vertex u, ompl::base::PlannerData::Graph::Vertex v) const;

            bool constructOptimalSolution(const base::PlannerTerminationCondition &ptc, const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                          ompl::base::PathPtr &solution);

            bool getNewGoalState(const base::PlannerTerminationCondition &ptc, Vertex& v);

            void copyPlannerDataToGraph();
        
            ompl::geometric::PRM::Vertex addMilestoneFromPlannerData(base::State *state, const std::vector<Vertex>& neighbors);

            base::PlannerData* pd_{nullptr};
            
            double min_sample_connection_distance_{0.8};
            double max_sample_clearance_{1.0};

            bool planner_data_set_{false};
        };
    }
}

#endif
