#pragma once

#include "data_fusion_interface/meta/motion.h"
#include "data_fusion_interface/meta/position.h"
#include "data_fusion_interface/v2x_sensor.h"
#include "sensor_preprocessing_output_to_fusion.h"
#include "df_client.h"
#include "sensor_subscriber.h"
#include <iostream>
#include "v2x_preprocessing_output_to_fusion.h"
#include <pthread.h>
#include "fusion_handler/coordinate_transformation_handler.h"
#include "fusion_handler/data_classification_handler.h"
#include "fusion_handler/time_synchronization_handler.h"
#include "tooling/timesync/timesync.h"
#include "tooling/kalman/kalman.h"
#include "data_fusion_interface/fusion_object.h"
#include <utility>

const int PARAM_OldObjDeleteCount = 3;
const int PARAM_TEMP2InternalCount = 2;
const int PARAM_TempObjCount = 3;
const int PARAM_NewSensorOnlyObjFlag = 31;

class TrackMgmt{

private:
   // data_fusion::EgoVehicle egoVehData;
    v2x_preprocessing::HostVehInfo egoVehData;
    // bsm info from fd bus, and also storage the obstacles after time sync and coordinate trans
    std::vector<v2x_preprocessing::BSM> bsmList;
    // obsctacle info from subscriber, and also storage the obstacles after time sync and coordinate trans
    sensor_preprocessing::s_Obstacles obsList;
    // current fusion data in track list
    data_fusion::FusionObjects trackList;



public:
    // get veh data from pos_server and obu_client
    void setVehData(DFClient &obu_client, DFClient &pos_client);
   // resevered function, used when set variables to private
    v2x_preprocessing::HostVehInfo getVehData();
    void setV2XData(DFClient &obu_client);
    // resevered function, used when set variables to private
    std::vector<v2x_preprocessing::BSM> getV2xData();
    // get sensor data from Subsciber and storage locally
    void setSensorData(SensorSubscriber &sensorSub);
    // resevered function, used when set variables to private
    sensor_preprocessing::s_Obstacles getSensorData();
    // initial data storage 
    void clearV2xData();
    void clearSensorData();
    // update Track info: update the obj info of traclist, do feature fusion
    // update the un-matched predicated(no measured data) obj with motion update
    void updateTrackObj(int TrackListIndex);
    // update the sensor only data
    void updateTrackObj(int SensorListindex, int TrackListIndex);
    // update the pair obj into track list
    void updateTrackObj(std::pair<int, int> matchTrackPair, int FusionVehindex);
    // update the whole track list
    void updateTrackList();
    // internal data transmission before send out
    void transDataFusionRet2Client(DFClient &obu_client);
    // check if the sensor only obj is new obj or not, return value is the index of TrackList
    int isSensorOnlyObjInTrackList(int index);
    // check if the fusion obj is new obj or not, return value is index of TrackLi
    int isMultSourceObjInTrackList(std::pair<int, int> matchIdPair);
    // create a new sensor only obj in track list: sensorList Index
    void createNewObjInTrackList(int index);
    // create a temp fusion obj in track list: bsmlist Index, sensorList Index, 
    void createTempObjInTrackList(std::pair<int, int> matchPair);
    // delete obj in Track Lis
    void deleteObjFromTrackList(int ObjIndex);
    // data fusion main thread
    void thread_datafusion(DFClient &obu_client, DFClient &pos_client, 
                            SensorSubscriber &sensorSub, 
                            data_fusion::V2XCoordinateTransformation *doV2xCoordinateTransformation,
                            data_fusion::SensorCoordinateTransformation *doSensorCoordinateTransformation,
                            data_fusion::V2XTimeSynchronization *doV2xTimeSynchronization,
                            data_fusion::SensorTimeSynchronization *doSensorTimeSynchronization,
                            data_fusion::DataClassification *dataClassification);



    // data classification: bsm & sensor data pair vector, including bsm only, 
    // int1: index of bsmList, int2:: index of obsList
    std::vector<std::pair<int, int>> matchIdPairList;
    // sensor only data, after data classification
    std::vector<int> sensorOnlyIdList;
    // // a vector pair to storage matched pair between current received bms.id/sensor.id and track list, no need dataAssociation
    // // int1: index of bsmList, int2: index of obsList, int3: index of fusionVehList;
    // std::vector<std::pair<std::pair<int, int>,int>> matchedMeasureAndTrackList;
    // before data association: a vector pair to storage un-matched pair that obj do not exist in track list, need do dataAssociation
    std::vector<std::pair<int,int>> unMatchMeasureDataListBefDaAss;
    // before data association: vector to storage un matched obj index of fusioned vehicle in track list
    std::vector<int> unMatchTrackDataListBefDaAss;
    //  after data association: matched table of measured data index(bsmList Index, sensorList Index) & predicated data index
    std::vector<std::pair<std::pair<int,int>, int>> dataAssociMatchTable;
    // after data association: un-matched table of measured data index
    std::vector<std::pair<int,int>> unMatchMeasureDataListAftDaAss;
    // after data association: un-matched table of measured data index
    std::vector<int> unMatchTrackDataListAftDaAss;
};

