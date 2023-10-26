#include <iostream>
#include <stdio.h>
#include "tooling/transform/transform.h"
#include "sensor_subscriber.h"
#include "tooling/timesync/timesync.h"
#include <iostream>
#include <common_base/CFdbContext.h>
#include <df_client.h>
#include "fusion_handler/coordinate_transformation_handler.h"
#include <time.h>
#include <unistd.h>
#include "mytimer.h"
#include "trackmgmt.h"
#include <thread>

std::shared_ptr<Subscriber<ObstaclesProtoSerializer>> obstacleSubscriber;
std::shared_ptr<Subscriber<LinesProtoSerializer>> lineSubscriber;

CBaseWorker main_worker;
const char* lane_server_url = "svc://coor_service";
const char* nbl_server_url = "svc://nl_service";
const char* pos_server_url = "svc://pos_sercer";

int main(int argc, char *argv[])
{
    // 第1步: OBU信息 
	FDB_CONTEXT->start();
    main_worker.start();
    DFClient *lane_client = new DFClient(lane_server_url, &main_worker);
    DFClient *obu_client = new DFClient(nbl_server_url, &main_worker);
    DFClient *pos_client = new DFClient(pos_server_url, &main_worker);
    lane_client->connect(lane_server_url);
    obu_client->connect(nbl_server_url);
    pos_client->connect(pos_server_url);
    CFdbMsgSubscribeList subscribe_list;
    pos_client->addNotifyItem(subscribe_list, NLPosition::POS_GNSS_DATA);
    pos_client->subscribe(subscribe_list);

    // 第2步: 订阅Sensor信息 
    SensorSubscriber sensorSubscriber;
    sensorSubscriber.InitSubscriber(obstacleSubscriber, lineSubscriber);

    TrackMgmt trackMgmt;
    data_fusion::V2XCoordinateTransformation v2xCoordTrans;
    data_fusion::SensorCoordinateTransformation sensorCoordTrans;
    data_fusion::V2XTimeSynchronization v2xTimeSync;
    data_fusion::SensorTimeSynchronization sensorTimeSync;
    data_fusion::DataClassification dataClassification;

    // *create a timer to cycle send out fusion data with 100 ms
    auto dataFusionfunc = [&]{
                trackMgmt.thread_datafusion(*obu_client,*pos_client,sensorSubscriber,
                &v2xCoordTrans,
                &sensorCoordTrans, 
                &v2xTimeSync,
                &sensorTimeSync,
                &dataClassification);
    };
    auto laneDataSend = std::bind(&DFClient::sendLaneInfo, lane_client, std::ref(sensorSubscriber._lines));
    // auto laneDataSend = [lane_client,&sensorSubscriber]{
    //     lane_client->sendLaneInfo(sensorSubscriber._lines);
    // };
    
    // *create a timer to cycle send out Lane data with 100 ms
    Timer dataFusion_timer(dataFusionfunc, 100);
    Timer laneDataSendOut_timer(laneDataSend, 100);

    // *start timer to send data out to fdbus server
    laneDataSendOut_timer.start();
    dataFusion_timer.start();

    // wait trigger to stop timer
    // laneDataSendOut_timer.stop();
    // dataFusion_timer.stop();
    while (1){}
    return 0;
}