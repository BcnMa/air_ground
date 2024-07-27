#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <driverless_common/structs.h>
#include <driverless_common/utils.h>
#include <vector>

namespace dcom {

GlobalPath::Ptr optimalPathPlanning(const GlobalPath::Ptr in_path, const VehicleState::ConstPtr v_state,
                                    const std::vector<Pose>& way_pts);









}//end namespace dcom

#endif // PATHPLANNING_H
