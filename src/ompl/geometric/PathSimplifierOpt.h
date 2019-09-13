#include "ompl/geometric/PathSimplifier.h"

namespace ompl
{
    namespace geometric
    {
        class PathSimplifierOpt : public PathSimplifier
        {
        public:
            /** \brief Create an instance for a specified space information. Optionally, a GoalSampleableRegion may be
            passed in to attempt improvements at the end of the path as well. */
            PathSimplifierOpt(base::SpaceInformationPtr si, const base::GoalPtr &goal, const base::OptimizationObjectivePtr& obj) :
                PathSimplifier(si, goal, obj) {}

            /** \brief Given a path, attempt to remove vertices from it while keeping the path valid. This is an
                iterative process that attempts to do "short-cutting" on the path. Connection is attempted between
                non-consecutive way-points on the path. If the connection is successful, the path is shortened by
                removing the in-between way-points. This function returns true if changes were made to the path.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to "short-cut" the path. If this value is set to 0 (the
                default), the number of attempts made is equal to the number of states in \e path.

                \param maxEmptySteps not all iterations of this function produce a simplification. If an iteration does
                not produce a simplification, it is called an empty step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification process terminates. If this value is set to 0 (the
                default), the number of attempts made is equal to the number of states in \e path.

                \param rangeRatio the maximum distance between states a connection is attempted, as a fraction relative
                to the total number of states (between 0 and 1).

            */
            bool reduceVertices(PathGeometric &path, unsigned int maxSteps = 0);
        };
    }
}
