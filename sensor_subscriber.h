#pragma once

#include <chrono>
#include <iostream>
#include <thread>
#include "communication/common/comm_log.h"
#include "communication/subscriber.h"
#include "communication/wait_set/scope_guard.h"
#include "schedulegroup/global_scheduler/global_scheduler.h"
#include "schedulegroup/errors.h"
#include <communication/common/types.h>
#include "communication/common/compiler_features.h"
#include "obstacle.pb.h"
#include "line.v2.pb.h"
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include "sensor_preprocessing_output_to_fusion.h"

using hobot::communication::COMM_CODE_OK;
using hobot::communication::CommAttr;
using hobot::communication::CompositeOption;
using hobot::communication::ErrorCode;
using hobot::communication::ErrorMsg;
using hobot::communication::EventType;
using hobot::communication::LinkInfo;
using hobot::communication::ParticipantAttr;
using hobot::communication::ScopeGuard;
using hobot::communication::Subscriber;
using hobot::communication::E2ErrorInfo;
using hobot::communication::E2EventType;
using hobot::communication::PROTOCOL_COMPOSITE;
using hobot::communication::PROTOCOL_DDS;
using hobot::communication::PROTOCOL_HYBRID;
using hobot::communication::PROTOCOL_INTRA;
using hobot::communication::PROTOCOL_INVALID;
using hobot::communication::PROTOCOL_PCIE;
using hobot::communication::PROTOCOL_SDIO;
using hobot::communication::PROTOCOL_SHM;
using hobot::communication::PROTOCOL_ZMQ_EPGM;
using hobot::communication::PROTOCOL_ZMQ_IPC;
using hobot::communication::PROTOCOL_ZMQ_TCP;
using hobot::communication::Message;
using hobot::communication::ProtoMsg;
using hobot::communication::ProtobufSerializer;
using hobot::communication::DataRef;
using hobot::communication::DefaultSerializer;

using hobot::message::TypeWrapper;
namespace ExtendInfoKey = hobot::message::ExtendInfoKey;


using ObstaclesProtoMsg = ProtoMsg<ObstacleProto::Obstacles>;
using ObstaclesProtoSerializer = ProtobufSerializer<ObstacleProto::Obstacles>;
using LinesProtoMsg = ProtoMsg<line_v2::Lines>;
using LinesProtoSerializer = ProtobufSerializer<line_v2::Lines>;

struct Args {
    int participant_id = 0;
    int protocol = PROTOCOL_INVALID;
    bool is_dynamic = false;
};

class SensorSubscriber
{
    public:
        line_v2::Lines _lines;
        sensor_preprocessing::s_Obstacles _ObsList;

    public:
        /**
         * 打印相关信息
         */
        void PrintUsage();

        /**
         * 验证命令行参数必须为非负整数
        */
        int Atoi(char *str_num);

        /**
         * 解析命令行参数
        */
        bool ParseCmd(int argc, char **argv, Args &args);

        /**
         * 匹配相关参数
        */
        bool MatchParticipantId(const Args &args, CommAttr &comm_attr);

        /**
         * ObstaclesProtoMsg 回调函数
        */
        void ObstaclesSubCallback(const std::shared_ptr<ObstaclesProtoMsg>& obstaclesProtoMsg);

        /**
         * LinesProtoMsg 回调函数
        */
        void LineSubCallback(const std::shared_ptr<LinesProtoMsg>& linesMsg);

        /**
         * Event 回调函数
        */
        void SubE2EventCallback(E2EventType &event,const std::shared_ptr<E2ErrorInfo> &info);

        /**
         * 初始化
        */
        void InitSubscriber(std::shared_ptr<Subscriber<ObstaclesProtoSerializer>>& obstacleSubscriber,std::shared_ptr<Subscriber<LinesProtoSerializer>>& lineSubscriber);
};


