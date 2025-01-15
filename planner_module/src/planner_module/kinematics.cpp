#include "planner_module/kinematics.hpp"

namespace cMRKinematics
{
    StateInformation state_info_; 
}

template class cMRKinematics::ArmKinematicsSolver<6>;
template class cMRKinematics::ArmKinematicsSolver<7>;
