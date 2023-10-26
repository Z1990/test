#pragma once

#include <iostream>
#include <string.h>
#include <vector>
#include <common_base/fdbus.h>
#include <common_base/CFdbProtoMsgBuilder.h>
#include "data_fusion_interface/fusion_object.h"
#include "v2x_preprocessing_output_to_fusion.h"
#include <time.h>
#include <nebulalink.vehicleinfo.pb.h>
#include <nebulalink.nlobuservice.pb.h>
#include "fusion_data_output.pb.h"
#include "lane.pb.h"
#include "line.v2.pb.h"
#include <vector>
#include "sensor_subscriber.h"
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>

class DFClient : public CBaseClient
{
public:
    DFClient(const char *server_name, CBaseWorker *worker = 0);

    static DFClient *getInstance()
    {
        return instance;
    }

    void getPositionData(const NLPosition::GNSSData &posData);

	void reqGetV2x(void);
	void getV2xfromOBU(const NLService::OBUs &obus);

    void reqGetHostVehInfo(void);
    void getHostVehfromOBU(const NLService::OBUHostInfo &hvInfo);

    void sendFusionData(const data_fusion::FusionObjects& fusionData);

    void sendLaneInfo(const line_v2::Lines &linesData);

    bool BsmUpdate();

    void getLaneInfo(SensorSubscriber *laneSub){
        this->_laneInfofromSensor = laneSub->_lines;
    }

    data_fusion::FusionObjects getFusionData(){
        return _fusionData;
    }

    void waitForOnReplyTrigger();

    void setOnReplyTrigger();

    // need move into private part
    // v2x_preprocessing::BSM g_BsmList[20];
    std::vector<v2x_preprocessing::BSM> g_BsmList;
    v2x_preprocessing::CWPosition g_hostVehPos;
    v2x_preprocessing::HostVehInfo g_hostVehInfo;
    line_v2::Lines _laneInfofromSensor;
    data_fusion::FusionObjects _fusionData;
    bool v2xReceivedFlag{false};

    // FLAG of client online or offline
    bool clientOnlineStatus{false};
    // Flag of onReply be triggered
    bool onReplyTirgger{false};
    // mutex for notify data fusion thread know onReply is triggered
    std::mutex onReplyMtx;
    std::condition_variable onReplyCv;


private:
    static DFClient *instance;
    
protected:
    void onOnline(FdbSessionId_t sid, bool is_first);

    void onOffline(FdbSessionId_t sid, bool is_last);

    void onReply(CBaseJob::Ptr &msg_ref);

    void onStatus(CBaseJob::Ptr &msg_ref, int32_t error_code, const char *description);

    /*
     * Implemented by client: called when server broadcast a message.
     * @iparam msg_ref: reference to a message. Using castToMessage()
     *      to convert it to CBaseMessage.
     */
    void onBroadcast(CBaseJob::Ptr &msg_ref);

};


