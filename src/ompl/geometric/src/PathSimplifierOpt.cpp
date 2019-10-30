#include "ompl/geometric/PathSimplifierOpt.h"

bool ompl::geometric::PathSimplifierOpt::reduceVertices(PathGeometric &path, unsigned int maxSteps)
{
    if (path.getStateCount() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = path.getStateCount()-2;

    bool result = false;
    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    std::vector<base::State *> &states = path.getStates();
    double total_cost_reduction = 0.0;

    double resolution = 0.001;

    for (unsigned int i = 0; i < maxSteps; i++)
    {
        int count = states.size();
        double best_cost_reduction = 0.0;
        int p_remove = -1;

        for (int j=1; j<count-1; j++)
        {
            auto p_old(std::make_shared<PathGeometric>(si));
            auto p_new(std::make_shared<PathGeometric>(si));
            p_old->append(states[j-1]);
            p_old->append(states[j]);
            p_old->append(states[j+1]);
            p_new->append(states[j-1]);
            p_new->append(states[j+1]);
            base::Cost cost_old = p_old->cost(obj_);
            base::Cost cost_new = p_new->cost(obj_);
            double cost_reduction = cost_old.value() - cost_new.value();
            
            if (cost_reduction > best_cost_reduction) {
                double distance = si->distance(states[j-1], states[j+1]);
                unsigned int num = (distance / resolution);
                base::State *test = si->allocState();
                bool valid = true;                
                for (unsigned int i=1; i<num; i++) {
                    si->getStateSpace()->interpolate(states[j-1], states[j+1], (double)i / (double)num, test);
                    if (!si->isValid(test)) {
                        valid = false;
                        break;
                    }
                }
                if (valid) {
                    p_remove = j;
                    best_cost_reduction = cost_reduction;
                }
            }
        }
        if (p_remove == -1) break;

        if (freeStates_)
            si->freeState(states[p_remove]);
        states.erase(states.begin() + p_remove);
        result = true;

        total_cost_reduction += best_cost_reduction;
    }
    return result;
}
