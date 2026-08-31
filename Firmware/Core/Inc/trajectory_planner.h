#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float p_start[3];
    float p_target[3];
    float q_target[3];
    float q_last[3];
    float duration;
    float elapsed_time;
    float settling_time;
    bool is_active;
} TrajectoryPlanner_t;

extern TrajectoryPlanner_t g_traj_planner;

void TrajectoryPlanner_Init(void);
bool TrajectoryPlanner_StartCtraj(const float target_xyz_mm[3], float duration_sec, const float current_q_deg[3]);
bool TrajectoryPlanner_Update(float dt_sec, float q_cmd_out[3], float qd_cmd_out[3], const float current_q_deg[3]);
void TrajectoryPlanner_Abort(void);

#endif /* TRAJECTORY_PLANNER_H_ */
