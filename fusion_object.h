#pragma once
#include "meta/motion.h"
#include "meta/position.h"
#include "meta/vehicle_size.h"
#include "meta/heading.h"
#include "meta/extension.h"

namespace data_fusion
{
    struct FusionObject
    {
        // flag of proceed in current cycle
        bool treatedFlag{false};

        // flag of temp object in track list
        bool tempObjFlag{false};

        // flag of new object which is create in cycle,  if the obj is not new, the flag shall be set as false
        bool newobjFlag{false};

        // flag of the obj is plan to be deleted in this cycle and will be remove at next cycle
        bool toBeDeleteFlag{false};

        // counter used for judge  if the obj should be detele
        int deleteCounter{0};

        // counter used for judge if the obj is a temp obj or not
        int tempObjCounter{0};

        // the flag of the obj is not Received in last cycle
        bool isNotReceivedFlag{false};

        // the flag of the obj is not Measured in last cycle
        bool isNotMeasuredFlag{false};


        /// temperary vehicle ID, from v2x message, the value 0 indicates that v2xId is unavailable
        u_int32_t v2xId{0};

        // temperary vehicle ID, from sensor Info, the value 0 indicates that id is unavailable
        u_int32_t sensorId{0};

        /// self-management, controlled by fusion component,the value 0 indicates that v2xId is unavailable
        int globalId{0};

        // Universal Time Coordinated, UTC timestamp
        int timestamp{0};

        /// relative position
        RelativePos relativePos;

        /// relative speed
        RelativeSpeed relativeSpeed;

        /// vehicle size
        VehSize vehSize;

        /// to confirm the vehicle has the cooperation ability or not
        bool cooperationAbility{false};

        /// heading of the vehicle
        Heading heading;

        /// object type
        ObjectType objType;
        
        /// datasource
        Datasource datasource;

        // global position
        GlobalPos globalPos;

        // absolute speed of object
        AbsoluteSpeed absoluteSpeed;
    };

    struct FusionObjects
    {
        /// <h4>Definition:</h4>
        /// It indicates the object count.<br/>
        /// 
        /// <h4>Data Range:</h4>
        /// INTEGER = (0, 10) {<br/>
        int totalCount{0};

        /// <h4>Definition:</h4>
        /// It indicates the vehicle object.<br/>
        FusionObject fusionVehicles[20];
    };

}