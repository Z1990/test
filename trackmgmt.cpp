#include "trackmgmt.h"
#include <vector>

void TrackMgmt::setVehData(DFClient &obu_client, DFClient &pos_client){
    egoVehData.timeStamp = obu_client.g_hostVehInfo.timeStamp;
    egoVehData.hvPosition.latitude = pos_client.g_hostVehPos.cwPos.latitude;
    egoVehData.hvPosition.longitude = pos_client.g_hostVehPos.cwPos.longitude;
    egoVehData.hvPosition.positioningState = pos_client.g_hostVehPos.cwPos.positioningState;
    egoVehData.hvAccel = obu_client.g_hostVehInfo.hvAccel;
    egoVehData.hvHeading = obu_client.g_hostVehInfo.hvHeading;
    egoVehData.hvSpeed = obu_client.g_hostVehInfo.hvSpeed;
}

v2x_preprocessing::HostVehInfo TrackMgmt::getVehData(){
    return this->egoVehData;
}

void TrackMgmt::setV2XData(DFClient &obu_client){
    this->bsmList = obu_client.g_BsmList;
}

std::vector<v2x_preprocessing::BSM> TrackMgmt::getV2xData(){
    return this->bsmList;
}

void TrackMgmt::setSensorData(SensorSubscriber& sensorSub){
    this->obsList = sensorSub._ObsList;
}

sensor_preprocessing::s_Obstacles TrackMgmt::getSensorData(){
    return this->obsList;
}

void TrackMgmt::clearV2xData(){
    this->bsmList.clear();
}

void TrackMgmt::clearSensorData(){
    if (this->obsList !=nullptr){
        delete this->obsList;
        this->obsList = nullptr;
    }
    this->obsList = new sensor_preprocessing::s_Obstacles();
}

void updateTrackObj(int TrackListIndex){

}

void TrackMgmt::updateTrackObj(int SensorListindex, int TrackListIndex){
    

}

void TrackMgmt::updateTrackObj(std::pair<int, int> matchTrackPair, int TrackListIndex){


}

void TrackMgmt::updateTrackList(){


}

void TrackMgmt::transDataFusionRet2Client(DFClient &obu_client){
    // use un-fusion data temperary to do test
    obu_client._fusionData.totalCount = this->obsList.obstacle_num;
    for (int i = 0; i <this->obsList.obstacle_num ; i++){
        obu_client._fusionData.fusionVehicles[i].v2xId = 0;
        obu_client._fusionData.fusionVehicles[i].sensorId = obsList.obstacle[i].id;
        obu_client._fusionData.fusionVehicles[i].relativePos.x = obsList.obstacle[i].vehWorldInfo.position_x;
        obu_client._fusionData.fusionVehicles[i].relativePos.y = obsList.obstacle[i].vehWorldInfo.position_y;
        // ...
    }
    std::cout << "fusion result: total count" << obu_client._fusionData.totalCount << std::endl;
}

int TrackMgmt::isSensorOnlyObjInTrackList(int index){
    // traversa the obj which marked newObj == false





}

int TrackMgmt::isMultSourceObjInTrackList(std::pair<int, int> matchIdPair){
    // traversa the obj which marked newObj == false

}

void TrackMgmt::createNewObjInTrackList(int index){
    trackList.totalCount++;
    trackList.fusionVehicles[trackList.totalCount].datasource = data_fusion::Datasource::sensor_only;
    // set value according to obsList[index]

}

void TrackMgmt::createTempObjInTrackList(std::pair<int, int> matchPair){
    this->trackList.totalCount++;
    this->trackList.fusionVehicles[this->trackList.totalCount-1].tempObjFlag = true;
    this->trackList.fusionVehicles[this->trackList.totalCount-1].newobjFlag = true;
    this->trackList.fusionVehicles[this->trackList.totalCount-1].treatedFlag = true;
    this->trackList.fusionVehicles[this->trackList.totalCount-1].datasource = data_fusion::Datasource::fusion;
    //...
}

void TrackMgmt::deleteObjFromTrackList(int ObjIndex){
    // set toBeDeleteFlag = true
    this->trackList.fusionVehicles[ObjIndex].treatedFlag = true;
    this->trackList.fusionVehicles[ObjIndex].toBeDeleteFlag = true;

}

void TrackMgmt::thread_datafusion(  DFClient& obu_client, DFClient& pos_client, 
                                    SensorSubscriber& sensorSub, 
                                    data_fusion::V2XCoordinateTransformation* v2xCoordTrans,
                                    data_fusion::SensorCoordinateTransformation* sensorCoordTrans,
                                    data_fusion::V2XTimeSynchronization* v2xTimeSync,
                                    data_fusion::SensorTimeSynchronization* sensorTimeSync,
                                    data_fusion::DataClassification *dataClassification){
    // judge if obu_server is disconnected, if yes, break; if no: continue:
    if (obu_client.clientOnlineStatus == false){
        return;
    }
    else{
        // get veh info from pos_client & obu_client
        obu_client.reqGetHostVehInfo();
        this->setVehData(obu_client,pos_client);

        // get v2x data from obu_client
        obu_client.reqGetV2x();
        // wait obu_client onReply trigger
        obu_client.setOnReplyTrigger();
        obu_client.waitForOnReplyTrigger();
        if (!obu_client.onReplyTirgger){
            std::this_thread::sleep_for(std::chrono::microseconds(30));
        }

        // if receive v2x data
        if (obu_client.v2xReceivedFlag == true){
            this->setV2XData(obu_client);
            // get sensor data from subscriber
            this->setSensorData(sensorSub);
            // do coord Trans & time sync
            long timestamp = getCurrentTimeStamp();
            for (std::vector<v2x_preprocessing::BSM>::iterator iterator = pos_client.g_BsmList.begin();iterator!=pos_client.g_BsmList.end();iterator++){
                v2xCoordTrans->doCoordinateTransformation(pos_client.g_hostVehInfo,*iterator);
                v2xTimeSync->doTimeSynchronization(timestamp,*iterator);
            }

            for (int index = 0; index < sensorSub._ObsList.obstacle_num; index ++){
                sensor_preprocessing::s_obstacle obs = sensorSub._ObsList.obstacle[index];
                sensorCoordTrans->doCoordinateTransformation(pos_client.g_hostVehInfo,obs);
                // time sync

            }
        }
        // if do not receive v2x data
        else {
            this->clearV2xData();
            // get sensor data from subscriber
            this->setSensorData(sensorSub);
            // do coord Trans & time sync
            for (int index = 0; index < sensorSub._ObsList.obstacle_num; index ++){
                sensor_preprocessing::s_obstacle obs = sensorSub._ObsList.obstacle[index];
                sensorCoordTrans->doCoordinateTransformation(pos_client.g_hostVehInfo,obs);
                // time sync
            }

        }
        // do data classification
        dataClassification->doDataclassification(this->getV2xData(),
                                                this->getSensorData(),
                                                this->matchIdPairList,
                                                this->sensorOnlyIdList);

        // handle sensor only data
        // travesal the object id of track list, judge if object is new or not
        for (auto it = sensorOnlyIdList.begin(); it != sensorOnlyIdList.end(); it++){
            int tempTrackListIndex = isSensorOnlyObjInTrackList(*it);
            if(tempTrackListIndex == PARAM_NewSensorOnlyObjFlag){
                // obj is not in track list
                createNewObjInTrackList(*it);
            }
            else{
                // in the list, set temp_flag = false
                // set deleteCounter = 0
                this->trackList.fusionVehicles[tempTrackListIndex].tempObjFlag = false;
                this->trackList.fusionVehicles[tempTrackListIndex].tempObjCounter = 0;
                this->trackList.fusionVehicles[tempTrackListIndex].deleteCounter = 0;
                updateTrackObj(*it, tempTrackListIndex);
            }

            // check if there is some obj which in track list, but not in current sensorOnlyList
            // && the obj is right close to right/left position of HV
            // if yes, keep the obj in track list and update the motion status acoordingly

        }

        // handle v2x&sensor data, including v2x only data
        // reset unMatchMeasureDataListBefDaAss & unMatchTrackDataListBefDaAss before use
        unMatchMeasureDataListBefDaAss.clear();
        unMatchTrackDataListBefDaAss.clear();
        for (auto it = matchIdPairList.begin(); it !=matchIdPairList.end();it++){
            // compare with track list objects(trackList) to judge if the object in the track liist
            int tempTrackListIndex = isMultSourceObjInTrackList(*it);
            if (tempTrackListIndex != 0){
                // do KF update

                // track list update
                // set delteCounter to 0;
                this->trackList.fusionVehicles[tempTrackListIndex].deleteCounter = 0;
                trackList.fusionVehicles[tempTrackListIndex].treatedFlag = true;
                updateTrackObj(*it,tempTrackListIndex);
            }
            else{
                // extract the un-matched pair to a new vector, and also need marked the un-matched objs in track mgmt list
                unMatchMeasureDataListBefDaAss.push_back(*it);
                unMatchTrackDataListBefDaAss.push_back(tempTrackListIndex);
            }
            
        }
        // do data association for un-matched vector pair
        // do data KF predication for unMatchTrackDataList

        // reset necessary data before use
        dataAssociMatchTable.clear();
        unMatchMeasureDataListAftDaAss.clear();
        unMatchTrackDataListAftDaAss.clear();
        // dataAssociation algrithm to get matched table of measured data index & predicated data index, and un-matched


        // traverse all the items in unmatched table
        // for unmatced predicated value: unMatchTrackDataListAftDaAss
        for (auto it = unMatchTrackDataListAftDaAss.begin(); it != unMatchTrackDataListAftDaAss.end(); it++){
            if (trackList.fusionVehicles[*it].isNotReceivedFlag == false){
                trackList.fusionVehicles[*it].isNotReceivedFlag = true;
                trackList.fusionVehicles[*it].deleteCounter = 1;
            }
            else{
                trackList.fusionVehicles[*it].deleteCounter++;
            }
            
            if (trackList.fusionVehicles[*it].deleteCounter == PARAM_OldObjDeleteCount){
                // delete the object
                trackList.fusionVehicles[*it].treatedFlag = true;
                trackList.fusionVehicles[*it].toBeDeleteFlag = true;
                break;
            }
            else{
                // update obj by predication value
                // KF update?

                trackList.fusionVehicles[*it].treatedFlag = true;
                updateTrackObj(*it);
            }   
        }
        // for unmatched measured value: unMatchMeasureDataListAftDaAss
        for (auto it = unMatchMeasureDataListAftDaAss.begin(); it!= unMatchMeasureDataListAftDaAss.end();it++){
            createTempObjInTrackList(*it);
            updateTrackObj(*it,trackList.totalCount-1);
        }
        // traverse all the items in matched table
        for (auto it = dataAssociMatchTable.begin(); it !=dataAssociMatchTable.end();it++){
            auto trackListIndex = (*it).second;
            if (trackList.fusionVehicles[trackListIndex].tempObjFlag){
                // continue 3 times judgement
                if (trackList.fusionVehicles[trackListIndex].isNotMeasuredFlag == false){
                    trackList.fusionVehicles[trackListIndex].isNotMeasuredFlag = true;
                    trackList.fusionVehicles[trackListIndex].tempObjCounter = 1;
                }
                else{
                    trackList.fusionVehicles[trackListIndex].tempObjCounter++;
                }

                if (trackList.fusionVehicles[trackListIndex].tempObjCounter == PARAM_TEMP2InternalCount){
                    // temp object change to real object
                    // KF update

                    trackList.fusionVehicles[trackListIndex].deleteCounter = 0;
                    trackList.fusionVehicles[trackListIndex].tempObjFlag = false;
                    trackList.fusionVehicles[trackListIndex].treatedFlag = true;
                    updateTrackObj((*it).first,(*it).second);
                }
                else{
                    // still keep the obj temp
                    // KF update

                    trackList.fusionVehicles[trackListIndex].deleteCounter = 0;
                    trackList.fusionVehicles[trackListIndex].treatedFlag = true;
                    updateTrackObj((*it).first,(*it).second);
                }
            }
            else{
                // the obj is in track list
                // do KF update

                trackList.fusionVehicles[trackListIndex].deleteCounter = 0;
                trackList.fusionVehicles[trackListIndex].treatedFlag = true;
                updateTrackObj((*it).first,(*it).second);
            }
        }

        // send data fusion result out
        this->updateTrackList();
        this->transDataFusionRet2Client(obu_client);
        obu_client.sendFusionData(obu_client._fusionData);  
    }
    
    
}