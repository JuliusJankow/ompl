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

            void insertStartAndGoalStates();

            void constructRoadmap(const unsigned int max_number_milestones);

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        protected:
            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
                This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(ompl::base::PlannerData::Graph::Vertex u, ompl::base::PlannerData::Graph::Vertex v) const;

            void copyPlannerDataToGraph();
        
            ompl::geometric::PRM::Vertex addMilestoneFromPlannerData(base::State *state, const std::vector<Vertex>& neighbors);

            base::PlannerData* pd_;
            
            bool isEdgeWeightsUpdated{false};
        };
    }
}

#endif
