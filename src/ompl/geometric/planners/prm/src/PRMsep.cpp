#include "ompl/geometric/planners/prm/PRMsep.h"
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>

#define foreach BOOST_FOREACH

ompl::geometric::PRMsep::PRMsep(const base::SpaceInformationPtr &si) : ompl::geometric::PRM(si, false)
{
    setName("PRMsep");
    //OMPL_INFORM("Using approximate solution, heuristic cost-to-go is %f", diff.value());
    OMPL_INFORM("PRMsep constructor");
}

ompl::geometric::PRMsep::PRMsep(const base::SpaceInformationPtr &si, base::PlannerData* pd) : PRMsep(si)
{
    setPlannerData(pd);
}

void ompl::geometric::PRMsep::setPlannerData(base::PlannerData* pd)
{
    pd_ = pd;
    OMPL_INFORM("Planner data set.");
}

void ompl::geometric::PRMsep::setup()
{
    PRM::setup();
    
    if(pd_ != nullptr) {
        copyPlannerDataToGraph();
    }

    OMPL_INFORM("PRMsep set up.");
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

void ompl::geometric::PRMsep::constructRoadmap(const unsigned int max_number_milestones)
{
    clear();

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    OMPL_INFORM("PRMsep constructing roadmap...");

    //std::vector<ompl::geometric::PRM::Vertex> milestone_list;
    base::State *workState = si_->allocState();

    // collect milestones and add them to the graph without connecting them to each other 
    unsigned int counter=0;
    while(counter < max_number_milestones) {
        bool found = false;
        unsigned int attempts = 0;
        
        while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found) {
            found = sampler_->sample(workState);
            attempts++;
        }
        if(found) {
            // milestone_list.push_back(addMilestoneWoConnecting(si_->cloneState(workState)));
            addMilestone(si_->cloneState(workState));
            counter++;
        }
        else {
            break;
        }
    }

    si_->freeState(workState);

    OMPL_INFORM("PRMsep constructed roadmap.");
}

void ompl::geometric::PRMsep::insertStartAndGoalStates()
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

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(base::timedPlannerTerminationCondition(1.0)) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return;
        }
    }

    std::cout << "Found " << startM_.size() << " start states and " << goalM_.size() << " goal states." << std::endl;
}

ompl::base::PlannerStatus ompl::geometric::PRMsep::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("PRMsep searching for an optimal path...");

    insertStartAndGoalStates();
    
    ompl::base::PlannerData::Graph::Vertex start = startM_[0];// = boost::vertex(pd_->getStartIndex(0), graph); // replace by start state from pdef
    ompl::base::PlannerData::Graph::Vertex goal = goalM_[0];

    std::lock_guard<std::mutex> _(graphMutex_);
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    try
    {
        boost::astar_search(g_, start,
            [this, goal](ompl::base::PlannerData::Graph::Vertex v) { return costHeuristic(v, goal); },
            boost::predecessor_map(prev).
            distance_compare([this](ompl::base::Cost c1, ompl::base::Cost c2) { return opt_->isCostBetterThan(c1, c2); }).
            distance_combine([this](ompl::base::Cost c1, ompl::base::Cost c2) { return opt_->combineCosts(c1, c2); }).
            distance_inf(opt_->infiniteCost()).
            distance_zero(opt_->identityCost()));
    }
    catch (...)
    {
        std::cout << "Shit happened";
    }

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");

    auto p(std::make_shared<PathGeometric>(si_));
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
        p->append(stateProperty_[pos]);
    p->append(stateProperty_[start]);
    p->reverse();

    // print the path to screen
    p->print(std::cout);
    std::cout << "Found stored solution with " << p->getStateCount() << " states and length " << p->length() << std::endl;
}

ompl::base::Cost ompl::geometric::PRMsep::costHeuristic(Vertex v, Vertex goal) const
{
    return opt_->motionCostHeuristic(stateProperty_[v], stateProperty_[goal]); // use cost-to-go heuristic
}
