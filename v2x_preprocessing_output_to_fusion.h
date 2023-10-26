#pragma once

#include <nebulalink.vehicleinfo.pb.h>
#include <nebulalink.nlobuservice.pb.h>
#include <nebulalink.position.pb.h>
#include <nebulalink.common.pb.h>
#include <string>
#include "data_fusion_interface/meta/position.h"
#include "data_fusion_interface/meta/heading.h"
#include "data_fusion_interface/meta/motion.h"

namespace v2x_preprocessing
{
   struct Pos{
        double longitude{0}; // unit: degree
        double latitude{0}; // unit: degree
        enum PositioningState {
            // 0-POS_STATE_NO_POS; 		//无定位 
            POS_STATE_NO_POS = 0, 
            // 1-POS_STATE_SINGLE_POINT; //单点定位
            POS_STATE_SINGLE_POINT = 1, 
            // 2-POS_STATE_DGPS; 		//DGPS或SBAS星基增强
            POS_STATE_DGPS = 2, 
            // 4-POS_STATE_RTK_FIX; 		//RTK固定解
            POS_STATE_RTK_FIX = 4,
            POS_STATE_RTK_FLOAT = 5,
            // 6-POS_STATE_DR; 			//纯惯导航位推算
            POS_STATE_DR = 6 
        };
        PositioningState positioningState{POS_STATE_NO_POS}; 
    };

    // position data from cw_position, internal interface 6;
    struct CWPosition {
        Pos cwPos; // position from cw_position
        u_int32_t unixtime{0}; // unit: ms
        float gnssSpeed{0}; // unit: m/s
    };

    struct VehSizeInfo{
        u_int32_t vehClass{0}; // vehicle class, such as car, truck, bus, etc.
        float length{0}; // unit: m
        float width{0}; // unit: m
        float height{0}; // unit: m
    };

    // bsm message, get from NBL_service, internal interface 3
    struct BSM {
        u_int32_t bsmID{0}; // bsm message ID
        std::string deviceID{""}; // device ID of the sender
        u_int32_t timeStamp{0}; // unit: ms
        Pos rvPosition; // position of the remote vehicle
        float vehSpeed{0}; // unit: m/s
        float vehHeading{0}; // unit: degree
        float vehAccel{0}; // unit: m/s^2
        VehSizeInfo vehSizeInfo; // vehicle size information
        float distance2HV;

        data_fusion::RelativePos relativePos;
        data_fusion::RelativeSpeed relativeSpeed;
        data_fusion::Heading relativeHeading;
    };

  // host vehicle info, get from NBL_Service, internal interface 4
  struct HostVehInfo {
      u_int32_t timeStamp{0}; // unit: millisecond
      Pos hvPosition; // position of the host vehicle
      float hvSpeed{0}; // unit: m/s ,if speedCANValid is true, use speedCAN;
      float hvHeading{0}; // unit: degree ,if headingCANValid is true, use headingCAN;
      float hvAccel{0}; // unit: m/s^2
  };
}