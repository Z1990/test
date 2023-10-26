#pragma once
#include <iostream>

namespace sensor_preprocessing
{
    struct s_worldInfo {
        // obstacle position data in vcs coordinate system: obstacles.obstacle.world_info.position.x;
        float position_x{0.0};

        // obstacle position data in vcs coordinate system: obstacles.obstacle.world_info.position.y;
        float position_y{0.0};

        // obstacle relative speed data in vcs: obstacles.obstacle.world_info.vel.vx;
        float vehSpeed_x{0.0};

        // obstacle relative speed data in vcs: obstacles.obstacle.world_info.vel.vy;
        float vehSpeed_y{0.0};
        
        // obstacle yaw: obstacles.obstacle.world_info.yaw;
        float yaw{0.0};
        
        // obstacle size: obstacles.obstacle.world_info.lenght;
        float vehLength{0.0};

        // obstacle size: obstacles.obstacle.world_info.width;
        float vehWidth{0.0};

        // obstacle size: obstacles.obstacle.world_info.height;
        float vehHeight{0.0};

        // obstacle relative accleration on vehicle's driving direction: obstacles.obstacle.world_info.acc
        float vehAcc{0.0};

        // obstacle relative accleration in vsc: obstacles.obstacle.world_info.acc_abs_world.ax;
        float vehAcc_ax{0.0};

        // obstacle relative accleration in vsc: obstacles.obstacle.world_info.acc_abs_world.ay;
        float vehAcc_ay{0.0};

        // obstacles's current lane position: obstacles.obstacle.world_info.curr_lane
        u_int32_t vehCurLane{0};

        // obstacle's motion state: obstacles.obstacle.world_info.motion_state
        u_int32_t vehMotionState{0};

        // it represents the distance from remote vehicle to host vehicle
        float distance2Veh;
    };

    struct s_obstacle {
        // obstacle id: Obstacles.obstacle.id
        u_int32_t id{0};

        // obstacle type: Obstacles.obstacle.type
        u_int32_t type{0};

        // perception confiendence
        u_int32_t conf{0};

        // detected frame number of this obstacle
        u_int32_t age{0};

        // obstacles valid flag: Obstacles.valid_info
        u_int32_t valid_info{0};

        // obstacles time stamp: Obstacles.mult_sensor_obs_fusion_status.time_stamp;
        u_int64_t time_stampe{0};

        // world info
        s_worldInfo vehWorldInfo;
    };

    struct s_Obstacles {
        // data source of obstacles: Obstacles.mult_sensor_obs_fusion_status.source;
        std::string mult_sensor_obs_fusion_status_source{""};

        // obstacles time stamp: Obstacles.mult_sensor_obs_fusion_status.time_stamp;
        u_int64_t time_stampe{0};

        // number of obstacle in one obstacles message
        u_int16_t obstacle_num{0};
        
        // assume maximum size is 30
        s_obstacle obstacle[30];
    };

};