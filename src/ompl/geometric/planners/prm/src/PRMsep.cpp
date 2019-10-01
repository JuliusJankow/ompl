#include "ompl/geometric/planners/prm/PRMsep.h"
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>

#define foreach BOOST_FOREACH

namespace ompl
{
    namespace magic
    {
        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 25;
        static const unsigned int FIND_INVALID_STATE_ATTEMPTS_WITHOUT_TERMINATION = 1000;
    }  // namespace magic
}  // namespace ompl

ompl::geometric::PRMsep::PRMsep(const base::SpaceInformationPtr &si) : ompl::geometric::PRM(si, false)
{
    setName("PRMsep");
}

ompl::geometric::PRMsep::PRMsep(const base::SpaceInformationPtr &si, base::PlannerData* pd) : PRMsep(si)
{
    setPlannerData(pd);
}

void ompl::geometric::PRMsep::clear()
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    clearQuery();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::PRMsep::setPlannerData(base::PlannerData* pd)
{
    pd_ = pd;
    setup_ = false;
    OMPL_INFORM("PRMsep: Planner data set.");
}

void ompl::geometric::PRMsep::setup()
{
    PRM::setup();
    
    if(pd_ != nullptr && !planner_data_set_) {
        copyPlannerDataToGraph();
        planner_data_set_ = true;
    }

    OMPL_INFORM("PRMsep: Set up.");
}

void ompl::geometric::PRMsep::copyPlannerDataToGraph()
{
    std::vector<Vertex> inserted_vertices;

    unsigned int nv = pd_->numVertices();

    for (unsigned int i = 0; i < nv; ++i)
    {
        std::map<unsigned int, const ompl::base::PlannerDataEdge *> nbrs;
        pd_->getEdges(i, nbrs);

        std::map<unsigned int, const ompl::base::PlannerDataEdge *>::const_iterator it;

        std::vector<Vertex> neighbors;
        for (it = nbrs.begin(); it != nbrs.end(); ++it) {
            if(i > it->first) {
                neighbors.push_back(inserted_vertices[it->first]);
            }
        }

        base::State* clone = si_->cloneState(pd_->getVertex(i).getState());
        inserted_vertices.push_back(addMilestoneFromPlannerData(clone, neighbors));
    }
}

ompl::geometric::PRM::Vertex ompl::geometric::PRMsep::addMilestoneFromPlannerData(base::State *state, const std::vector<Vertex>& neighbors)
{
    std::lock_guard<std::mutex> _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    foreach (Vertex n, neighbors) {
        totalConnectionAttemptsProperty_[m]++;
        totalConnectionAttemptsProperty_[n]++;
        successfulConnectionAttemptsProperty_[m]++;
        successfulConnectionAttemptsProperty_[n]++;
        const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
        const Graph::edge_property_type properties(weight);
        boost::add_edge(n, m, properties, g_);
        uniteComponents(n, m);
    }

    nn_->add(m);

    return m;
}

double ompl::geometric::PRMsep::debugFreeSpace(base::State *state)
{
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    if (sampler_->sample(state))
        return si_->getStateValidityChecker()->clearance(state);

    return 0.0;
}

void ompl::geometric::PRMsep::constructRoadmap(const unsigned int max_number_milestones)
{
    PRM::clear();

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    OMPL_INFORM("PRMsep: Construct roadmap...");

    base::State *workState = si_->allocState();

    const ompl::base::StateValidityCheckerPtr& svc = si_->getStateValidityChecker();

    // collect milestones and add them to the graph without connecting them to each other 
    unsigned int counter=0, rejection_counter=0;
    double sample_clearance;
    while(counter < max_number_milestones) {
        bool found = false;
        unsigned int attempts = 0;
        
        while (attempts < magic::FIND_INVALID_STATE_ATTEMPTS_WITHOUT_TERMINATION && !found) {
            if (sampler_->sample(workState)) {
                // check if any other Vertex is in hyper sphere around new vertex
                Vertex v = boost::add_vertex(g_);
                stateProperty_[v] = workState;
                
                // if in box, evaluate cost function and compare to d_obstacle. Check only for 25 nearest
                std::vector<Vertex> nn_list;
                nn_->nearestK(v, magic::DEFAULT_NEAREST_NEIGHBORS, nn_list);
                found = true;
                double distance;
                for (size_t i=0; i<nn_list.size() && found; i++) {
                    base::State* st = stateProperty_[nn_list[i]];
                    bool new_state = i==0;
                    found = svc->isValid(workState, distance, st, new_state);
                }
                if (!found) rejection_counter++;

                boost::remove_vertex(v, g_);
            }
            attempts++;
        }
        if(found) { 
            connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
            addMilestone(si_->cloneState(workState));
            counter++;
            if (counter%1000 == 0) {
                OMPL_INFORM("PRMsep: %d milestones and %d edges found. %d samples rejected meanwhile.", milestoneCount(), edgeCount(), rejection_counter);
                rejection_counter = 0;
            }
        }
        else {
            break;
        }
    }

    si_->freeState(workState);

    OMPL_INFORM("PRMsep: constructed roadmap with %d milestones and %d edges.", milestoneCount(), edgeCount());
}

void ompl::geometric::PRMsep::insertStartAndGoalStates(unsigned int number_goals)
{
    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart()) {
        startM_.push_back(addMilestone(si_->cloneState(st)));
    }

    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return;
    }

    // check all samples if they are in goal region
    /*foreach (const Vertex v, boost::vertices(g_)) {
        const base::State* st = stateProperty_[v];
        if (goal->isSatisfied(st) && goal->isStartGoalPairValid(st, stateProperty_[startM_[0]])) {
            goalM_.push_back(addMilestone(si_->cloneState(st)));
            if (goalM_.size() >= number_goals)
                break;
        }
    }*/

    /*if (goalM_.size() < number_goals) {
        const base::StateSpacePtr& space = si_->getStateSpace();
        OMPL_INFORM("PRMsep: Sample more goal states... At max %i new goals.", goal->maxSampleCount());
        for (unsigned int i=0; i<goal->maxSampleCount() && goalM_.size() < number_goals; i++)
        {
            const base::State* st = pis_.nextGoal(base::timedPlannerTerminationCondition(1.0));
            if (st)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }
    }
    if (goalM_.empty())
    {
        OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
        return;
    }*/

    OMPL_INFORM("PRMsep: Found %i start states and %i goal states.", startM_.size(), goalM_.size());
}

bool ompl::geometric::PRMsep::getNewGoalState(const base::PlannerTerminationCondition &ptc, Vertex& v)
{
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return false;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return false;
    }

    while (!ptc()) {
        const base::State* goal_state = pis_.nextGoal(ptc);
        if (goal_state && si_->isValid(goal_state)) {
            v = addMilestone(si_->cloneState(goal_state));
            return true;
        }
    }
    return false;
}

bool ompl::geometric::PRMsep::constructOptimalSolution(const base::PlannerTerminationCondition &ptc, const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                                       base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    base::Cost sol_cost(opt_->infiniteCost());
    Vertex goal;

    while (!ptc()) {
        if (getNewGoalState(ptc, goal)) {
            foreach (Vertex start, starts)
            {
                // we lock because the connected components algorithm is incremental and may change disjointSets_
                graphMutex_.lock();
                bool same_component = sameComponent(start, goal);
                graphMutex_.unlock();

                if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
                {
                    base::PathPtr p = constructSolution(start, goal); 
                    if (p)
                    {
                        base::Cost pathCost = p->cost(opt_);
                        OMPL_INFORM("PRMsep: Found solution with length %f and cost %f", p->length(), pathCost.value());
                        if (opt_->isCostBetterThan(pathCost, bestCost_))
                            bestCost_ = pathCost;
                        // Check if optimization objective is satisfied
                        if (opt_->isSatisfied(pathCost))
                        {
                            solution = p;
                            return true;
                        }
                        if (opt_->isCostBetterThan(pathCost, sol_cost))
                        {
                            solution = p;
                            sol_cost = pathCost;
                        }
                    }
                }
                else
                    OMPL_WARN("PRMsep: Start/Goal pair is not connected.");
            }
        }
    }

    /*foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                base::PathPtr p = constructSolution(start, goal); 
                if (p)
                {
                    base::Cost pathCost = p->cost(opt_);
                    OMPL_INFORM("PRMsep: Found solution with length %f and cost %f", p->length(), pathCost.value());
                    if (opt_->isCostBetterThan(pathCost, bestCost_))
                        bestCost_ = pathCost;
                    // Check if optimization objective is satisfied
                    if (opt_->isSatisfied(pathCost))
                    {
                        solution = p;
                        return true;
                    }
                    if (opt_->isCostBetterThan(pathCost, sol_cost))
                    {
                        solution = p;
                        sol_cost = pathCost;
                    }
                }
            }
            else
                OMPL_WARN("PRMsep: Start/Goal pair is not connected.");
        }
    }*/

    return solution.get() != nullptr;
}


ompl::base::PlannerStatus ompl::geometric::PRMsep::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("PRMsep: Search for an optimal path...");

    if (!isSetup())
        setup();

    if (boost::num_vertices(g_) == 0) {
        OMPL_INFORM("PRMsep: There is no roadmap available, construct a new one.");
        constructRoadmap(10000);
    }

    insertStartAndGoalStates(10);
    
    base::PathPtr sol;
    constructOptimalSolution(ptc, startM_, goalM_, sol);

    if (sol)
    {
        // sol->print(std::cout);
        OMPL_INFORM("PRMsep: Best solution has length %f and cost %f", sol->length(), sol->cost(opt_).value());
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);
    }

    return sol ? ompl::base::PlannerStatus::EXACT_SOLUTION : ompl::base::PlannerStatus::TIMEOUT;
}

ompl::base::Cost ompl::geometric::PRMsep::costHeuristic(Vertex v, Vertex goal) const
{
    return opt_->motionCostHeuristic(stateProperty_[v], stateProperty_[goal]); // opt_->motionCostHeuristic(stateProperty_[v], stateProperty_[goal]); // use cost-to-go heuristic
}
