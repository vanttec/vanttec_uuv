#ifndef __CHOOSE_SIDE_MISSION__
#define __CHOOSE_SIDE_MISSION__

#include <vanttec_uuv/GuidanceWaypoints.h>

typedef enum ChooseSideStates_E
{
    STANDBY = 0,
    SEARCH = 1,
    CALCULATE = 2,
    NAVIGATE = 3,
} ChooseSideStates_E;

typedef enum Side_E
{
    LEFT = 0,
    RIGHT = 1,
} Side_E;

class ChooseSideMission
{
    public:
        
        Side_E                  selected_side;
        ChooseSideStates_E      state_machine;

        vanttec_uuv::GuidanceWaypoints  desired_waypoints;
        geometry_msgs::Pose             current_position;

        ChooseSideMission();
        ~ChooseSideMission();

        void OnPoseReception();
        void On
        void 

        void Iteration();
};


#endif