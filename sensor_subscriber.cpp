#include "sensor_subscriber.h"

void SensorSubscriber::PrintUsage() {
    std::cout << "Usage:\n"
              << "     ./subscriber {ProtocolType}\n"
              << "     ./subscriber {ProtocolType} [participant_id]\n"
              << "     ./subscriber {ProtocolType} [dynamic]\n"
              << "note: this example only support following protocols:\n"
              << "protocol id : protocol type\n"
              << "         0  :  hybrid\n"
              << "         2  :  zmq tcp\n"
              << "         3  :  zmq epgm\n"
              << "         4  :  zmq ipc\n"
              << "         6  :  shared memory (not for hbmem)\n"
              << "         7  :  sdio\n"
              << "         8  :  pcie" << std::endl;
}

int SensorSubscriber::Atoi(char *str_num) {
    std::string int_max = std::to_string(INT_MAX);
    if (strlen(str_num) > int_max.size()) return -1;
    for (int i = 0; i < strlen(str_num); ++i) {
      if (str_num[i] < '0' || str_num[i] > '9') {
        return -1;
      }
    }
    if (strlen(str_num) == int_max.size()) {
      for (int i = 0; i < int_max.size(); ++i) {
        if (str_num[i] > int_max[i]) {
          return -1;
        }
      }
    }
    return atoi(str_num);
}

bool SensorSubscriber::ParseCmd(int argc, char **argv, Args &args) {
    if (argc < 2 || argc > 3) {
      PrintUsage();
      return false;
    }
    if ((args.protocol = Atoi(argv[1])) == -1) {
      std::cout << "invalid protocol type: " << argv[1] << std::endl;
      return false;
    }
    if (argc == 2) {
      args.is_dynamic = false;
      std::cout << "participant id hasn't been input, default value will be use" << std::endl;
      return true;
    } else {
      if (std::string(argv[2]) == "dynamic") {
        args.is_dynamic = true;
        return true;
      } else if ((args.participant_id = Atoi(argv[2])) != -1) {
        return true;
      } else {
        PrintUsage();
        return false;
      }
    }
  }

bool SensorSubscriber::MatchParticipantId(const Args &args, CommAttr &comm_attr) {
    switch (args.protocol) {
      case PROTOCOL_HYBRID:
        if (args.is_dynamic) {
          return true;
        }
        std::cout << "using protocol hybrid must turn on dynamic discovery"<< std::endl;
        return false;
      case PROTOCOL_INTRA:
        std::cout << "can not use protocol intra in this example, run " "intra_pub_sub" << std::endl;
        return false;
      case PROTOCOL_ZMQ_TCP:
        if (!args.is_dynamic) {
          if (args.participant_id == 0)
            comm_attr.participant_attrs_.push_back(ParticipantAttr{2});
          else
            comm_attr.participant_attrs_.push_back(ParticipantAttr{args.participant_id});
        }
        return true;
      case PROTOCOL_ZMQ_EPGM:
        if (!args.is_dynamic) {
          if (args.participant_id == 0)
            comm_attr.participant_attrs_.push_back(ParticipantAttr{7});
          else
            comm_attr.participant_attrs_.push_back(ParticipantAttr{args.participant_id});
        }
        return true;
      case PROTOCOL_ZMQ_IPC:
        if (!args.is_dynamic) {
          if (args.participant_id == 0)
            comm_attr.participant_attrs_.push_back(ParticipantAttr{1});
          else
            comm_attr.participant_attrs_.push_back(ParticipantAttr{args.participant_id});
        }
        return true;
      case PROTOCOL_DDS:
        std::cout << "this example do not support protocol DDS, please run " "corresponding fastdds example" << std::endl;
        return false;
      case PROTOCOL_SHM:
        if (args.is_dynamic) return true;
        std::cout << "using protocol shm must turn on dynamic discovery" << std::endl;
        return false;
      case PROTOCOL_SDIO:
        if (args.is_dynamic) {
          std::cout << "dynamic discovery do not support protocol sdio" << std::endl;
          return false;
        }
        if (args.participant_id == 0)
          comm_attr.participant_attrs_.push_back(ParticipantAttr{4});  // 4 is cp
        else
          comm_attr.participant_attrs_.push_back(ParticipantAttr{args.participant_id});
        return true;
      case PROTOCOL_PCIE:
        if (args.is_dynamic) {
          std::cout << "dynamic discovery do not support protocol pcie" << std::endl;
          return false;
        }
        if (args.participant_id == 0) {
          std::cout << "if you want to use ep2ep mode, input corresponding " "participant id, in default config file, it is 10" << std::endl;
          comm_attr.participant_attrs_.push_back(ParticipantAttr{9});
        } else {
          comm_attr.participant_attrs_.push_back(ParticipantAttr{args.participant_id});
        }
        return true;
      case PROTOCOL_COMPOSITE:
        comm_attr.composite_options.push_back(CompositeOption(PROTOCOL_INTRA));
        comm_attr.composite_options.push_back(CompositeOption(PROTOCOL_SHM));
        comm_attr.composite_options.push_back(CompositeOption(PROTOCOL_ZMQ_TCP));
        return true;
      default:
        std::cout << "invalid protocol type: " << args.protocol << std::endl;
        return false;
    }
}

void SensorSubscriber::LineSubCallback(const std::shared_ptr<LinesProtoMsg>& linesMsg)
{
    line_v2::Lines lines = linesMsg.get()->proto;
    this->_lines = lines;
    //std::cout << "received  msg info: \n " << "src timestamp =" << _lines.src_time_stamp() << std::endl;
    HSLOGM_D("LineSubCallback")<< "received  msg info: \n " << "src timestamp =" << _lines.src_time_stamp();
}

void SensorSubscriber::ObstaclesSubCallback(const std::shared_ptr<ObstaclesProtoMsg>& obstaclesProtoMsg) {
    // can be comment out before final release
    ObstacleProto::Obstacles obstacles = obstaclesProtoMsg.get()->proto;
    //this->_obstacles = obstacles;
    // std::cout << "received msg info: \n " << "cam_id =" << obstacles.cam_id() << std::endl;

    // assign the obstacle proto data to v2x_preprocessing_output::_obsList
    this->_ObsList.obstacle_num = obstacles.obstacle_size();
    this->_ObsList.mult_sensor_obs_fusion_status_source = obstacles.mult_sensor_obs_fusion_status().source();
    this->_ObsList.time_stampe = obstacles.mult_sensor_obs_fusion_status().time_stamp();
    for (int index = 0; index < obstacles.obstacle_size(); index ++){
      this->_ObsList.obstacle->age = obstacles.obstacle(index).valid_info();
      this->_ObsList.obstacle->conf = obstacles.obstacle(index).conf();
      this->_ObsList.obstacle->id = obstacles.obstacle(index).id();
      this->_ObsList.obstacle->type = obstacles.obstacle(index).type();
      this->_ObsList.obstacle->valid_info = obstacles.obstacle(index).valid_info();
      this->_ObsList.obstacle->vehWorldInfo.position_x = obstacles.obstacle(index).world_info().position().x();
      this->_ObsList.obstacle->vehWorldInfo.position_y = obstacles.obstacle(index).world_info().position().y();
      this->_ObsList.obstacle->vehWorldInfo.vehAcc = obstacles.obstacle(index).world_info().acc();
      this->_ObsList.obstacle->vehWorldInfo.vehAcc_ax = obstacles.obstacle(index).world_info().acc_abs_world().ax();
      this->_ObsList.obstacle->vehWorldInfo.vehAcc_ay = obstacles.obstacle(index).world_info().acc_abs_world().ay();
      this->_ObsList.obstacle->vehWorldInfo.vehCurLane = obstacles.obstacle(index).world_info().curr_lane();
      this->_ObsList.obstacle->vehWorldInfo.vehHeight = obstacles.obstacle(index).world_info().height();
      this->_ObsList.obstacle->vehWorldInfo.vehLength = obstacles.obstacle(index).world_info().length();
      this->_ObsList.obstacle->vehWorldInfo.vehWidth = obstacles.obstacle(index).world_info().width();
      this->_ObsList.obstacle->vehWorldInfo.vehMotionState = obstacles.obstacle(index).world_info().motion_state();
      this->_ObsList.obstacle->vehWorldInfo.vehSpeed_x = obstacles.obstacle(index).world_info().vel().vx();
      this->_ObsList.obstacle->vehWorldInfo.vehSpeed_y = obstacles.obstacle(index).world_info().vel().vy();
    }
    
}

void SensorSubscriber::SubE2EventCallback(E2EventType& event,const std::shared_ptr<E2ErrorInfo>& info) {
    //std::cout << "E2E check failed error code " << event << ", error info is " 
    //<< info->error_msg << ", participant info is " << info->link_info << std::endl;
    HSLOGM_E("SubE2EventCallback")<<"E2E check failed error code " << event << ", error info is " 
    << info->error_msg << ", participant info is " << info->link_info;
}

void SensorSubscriber::InitSubscriber(std::shared_ptr<Subscriber<ObstaclesProtoSerializer>>& obstacleSubscriber,std::shared_ptr<Subscriber<LinesProtoSerializer>>& lineSubscriber){

    Args args;
    args.participant_id = 2;
    args.protocol = 2;
    hobot::schedulegroup::GlobalSchedulerParam param;
    int32_t create_ret = hobot::schedulegroup::GlobalScheduler::Init(param);
    if (create_ret != hobot::schedulegroup::ErrorCode::OK) {
      //std::cout << "GlobalScheduler::Init failed" << std::endl;
      HSLOGM_E("InitSubscriber")<< "GlobalScheduler::Init failed";
      return;
    }
    hobot::communication::Init("communication.json");
    ScopeGuard gurad([]() {
      hobot::communication::DeInit();
      hobot::schedulegroup::GlobalScheduler::DeInit();
    });
    std::string version = hobot::communication::GetVersion();
    //std::cout << "communication version is " << version << std::endl;
    HSLOGM_I("InitSubscriber") << "communication version is " << version; 
    CommAttr comm_attr;
    MatchParticipantId(args, comm_attr);

    std::function<void(E2EventType &,const std::shared_ptr<E2ErrorInfo>&)> e2eSubCallback = 
    std::bind(&SensorSubscriber::SubE2EventCallback,this,std::placeholders::_1,std::placeholders::_2);
    comm_attr.e2e_callback = e2eSubCallback;
    
    // 订阅 ObstaclesProtoMsg
    std::string topic = "multisensor_fusion_objects";
    //std::shared_ptr<Subscriber<ObstaclesProtoSerializer>> obstacleSubscriber;
    std::function<void(const std::shared_ptr<ObstaclesProtoMsg> &)> obsCallback = 
    std::bind(&SensorSubscriber::ObstaclesSubCallback,this,std::placeholders::_1);
    obstacleSubscriber = Subscriber<ObstaclesProtoSerializer>::New(comm_attr, topic+"*", 0, obsCallback, args.protocol);
    if (!obstacleSubscriber) {
      //std::cout << " create obstacleSubscriber failed" << std::endl;
      HSLOGM_E("InitSubscriber")<<" create obstacleSubscriber failed";
      return;
    }
    // 订阅 LinesProtoMsg
    std::string laneline_v2_topic = "laneline_v2";
    //std::shared_ptr<Subscriber<LinesProtoSerializer>> lineSubscriber;
    std::function<void(const std::shared_ptr<LinesProtoMsg> &)> lineCallback = 
    std::bind(&SensorSubscriber::LineSubCallback,this,std::placeholders::_1);
    lineSubscriber = Subscriber<LinesProtoSerializer>::New(comm_attr, laneline_v2_topic+"*", 0, lineCallback, args.protocol);
    if (!lineSubscriber) {
      std::cout << " create lineSubscriber failed" << std::endl;
      HSLOGM_E("InitSubscriber")<<" create lineSubscriber failed";
      return;
    }
}