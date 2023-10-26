#include "df_client.h"



DFClient *DFClient::instance = NULL;

DFClient::DFClient(const char *server_name, CBaseWorker *worker) 
        : CBaseClient(server_name, worker)
{
    instance = this;
}

void DFClient::onOnline(FdbSessionId_t sid, bool is_first)
{
    printf("data fusion client service is online(sid:%d ; is_first:%s)\n", sid, (is_first ? "true" : "false"));
    std::cout << "online client: " << this->name() << std::endl;
    this->clientOnlineStatus = true;

}

void DFClient::onOffline(FdbSessionId_t sid, bool is_last)
{
    printf("data fusion client service is offline(sid:%d ; is_last:%s)\n", sid, (is_last ? "true" : "false"));
    this->clientOnlineStatus = false;
}

void DFClient::onReply(CBaseJob::Ptr &msg_ref)
{
    // change CBaseJob class to CBaseMessage class
	auto msg = castToMessage<CBaseMessage *>(msg_ref);										
	switch(msg->code())
	{

		case NLService::REQ_GET_OBUS_FROM_V2X:
		{   
            
			NLService::OBUs obus;
			CFdbProtoMsgParser parser(obus);
			msg->deserialize(parser);											
			if (!((msg->getPayloadSize() == 0) && (msg->getRawDataSize() != 0))){
                // add mutex
                this->v2xReceivedFlag = false;
                this->onReplyTirgger = true;
                break;
            }
            this->onReplyTirgger = true;
            getV2xfromOBU(obus);
		}
		break;
        case NLService::REQ_GET_OBUHOSTINFO:
        {
            NLService::OBUHostInfo hostVeh;
            CFdbProtoMsgParser parser(hostVeh);
            msg->deserialize(parser);
            this->onReplyTirgger = true;
            getHostVehfromOBU(hostVeh);
        }
        break;
        // case NLService::REQ_SET_LANE_INFO:
        // { 
        //     std::cout << "lane requestor: " << this->name() << std::endl;
        //     if (this->name()=="svc:://lane_client"){
        //         // std::cout <<"lane requestor: "<< this->name() << std::endl;
        //         sendLaneInfo(this->getLaneInfo());
        //     }
        // }
        // break;
        // case NLService::REQ_SET_FILTERED_VEHS:
        // {   
        //     std::cout << "fusion requestor: " << this->name() << std::endl;
        //     if (this->name() == "svc://obu_client"){
        //         sendFusionData(this->getFusionData());
        //     }
        //     // std::cout <<"fusion data requestor: "<< this->name() << std::endl;
        // }
        // break;
		default:
        {   
			printf("onReply unknow type %d\n",msg->code());
            std::cout << "client name: " << this->name() << std::endl;
        }
	    break;
	    }
}

void DFClient::onStatus(CBaseJob::Ptr &msg_ref, int32_t error_code, const char *description)
{
    CFdbMessage *msg = castToMessage<CFdbMessage *>(msg_ref);
    printf("msg->code(%d), error_code(%d), description: %s", msg->code(), error_code, description);
}

// receive gnss data from cw_position server
// before call position message, need subscribe in main.cpp: client.subscribe(NLPosition::POS_GNSS_DATA)

void DFClient::onBroadcast(CBaseJob::Ptr &msg_ref){
	auto msg = castToMessage<CBaseMessage *>(msg_ref);										

	switch(msg->code())
	{
		case NLPosition::POS_GNSS_DATA:
		{
			NLPosition::GNSSData posData;
			CFdbProtoMsgParser parser(posData);
			msg->deserialize(parser);												
			getPositionData(posData);
		}
		break;

		default:
        {
			printf("onBroadcast: unknow type %d\n",msg->code());
        }
	    break;
	}
}

void DFClient::getPositionData(const NLPosition::GNSSData &posData) {
    g_hostVehPos.cwPos.longitude = posData.position3d().longitude();
    g_hostVehPos.cwPos.latitude = posData.position3d().latitude();
    // g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::PositioningState(posData.pos_state());
    switch (posData.pos_state()){
        case 0:
            g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::POS_STATE_NO_POS;
            break;
        case 1:
            g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::POS_STATE_SINGLE_POINT;
            break;
        case 2:
            g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::POS_STATE_DGPS;
            break;
        case 4:
            g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::POS_STATE_RTK_FIX;
            break;
        case 5:
            g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::POS_STATE_RTK_FLOAT;
            break;
        case 6:
            g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::POS_STATE_DR;
            break;
        default:
            g_hostVehPos.cwPos.positioningState = v2x_preprocessing::Pos::POS_STATE_NO_POS;
            break;
    }
    g_hostVehPos.unixtime = posData.timeinfo().unixtime();
    g_hostVehPos.gnssSpeed = posData.speed();
    
    // std::cout << "***********************\n" << getTime() << ": Get CW_Position Data, position:  " << this->g_hostVehPos.cwPos.latitude <<", "<< this->g_hostVehPos.cwPos.longitude<<std::endl;
    // std::cout  << getTime() << ": Get CW_Position Data, unixtime:  " << this->g_hostVehPos.unixtime << "\n*********" << std::endl;
    
    std::cout << "***********************\n" << ": Get CW_Position Data, position:  " << this->g_hostVehPos.cwPos.latitude <<", "<< this->g_hostVehPos.cwPos.longitude<<std::endl;
    std::cout << ": Get CW_Position Data, unixtime:  " << this->g_hostVehPos.unixtime << "\n*********" << std::endl;
}

void DFClient::reqGetV2x(void)
{
   this->invoke(NLService::REQ_GET_OBUS_FROM_V2X); 
   this->onReplyTirgger = false;
}

void DFClient::getV2xfromOBU(const NLService::OBUs& obus)
{
    // init vector<BSM> to ensure received bsm data is update to date
    g_BsmList.clear();
 
    // variable assignment
    int obu_size = obus.obu_size();
    for (int index = 0; index < obu_size ; index ++){
        v2x_preprocessing::BSM temp_Bsm;
        if (obus.obu(index).has_gnss()){
            temp_Bsm.timeStamp = obus.obu(index).gnss().timebase().miilsecond();
            temp_Bsm.rvPosition.latitude = obus.obu(index).gnss().point().latitude();
            temp_Bsm.rvPosition.longitude = obus.obu(index).gnss().point().longitude();
            switch (obus.obu(index).gnss().positioningstate()){
                case 0:
                    temp_Bsm.rvPosition.positioningState = v2x_preprocessing::Pos::POS_STATE_NO_POS;
                    break;
                case 1:
                    temp_Bsm.rvPosition.positioningState = v2x_preprocessing::Pos::POS_STATE_SINGLE_POINT;
                    break;
                case 2:
                    temp_Bsm.rvPosition.positioningState = v2x_preprocessing::Pos::POS_STATE_DGPS;
                    break;
                case 4:
                    temp_Bsm.rvPosition.positioningState = v2x_preprocessing::Pos::POS_STATE_RTK_FIX;
                    break;
                case 5:
                    temp_Bsm.rvPosition.positioningState = v2x_preprocessing::Pos::POS_STATE_RTK_FLOAT;
                    break;
                case 6:
                    temp_Bsm.rvPosition.positioningState = v2x_preprocessing::Pos::POS_STATE_DR;
                    break;
                default:
                    temp_Bsm.rvPosition.positioningState = v2x_preprocessing::Pos::POS_STATE_NO_POS;
                    break;
            }
            temp_Bsm.vehSpeed = obus.obu(index).gnss().speed();
            temp_Bsm.vehHeading = obus.obu(index).gnss().heading();
            temp_Bsm.vehAccel = obus.obu(index).gnss().accel();
        }

        if (obus.obu(index).has_vehinfodesc()){      
            temp_Bsm.vehSizeInfo.length = obus.obu(index).vehinfodesc().length();
            temp_Bsm.vehSizeInfo.width = obus.obu(index).vehinfodesc().width();
            temp_Bsm.vehSizeInfo.height = obus.obu(index).vehinfodesc().height();
            temp_Bsm.vehSizeInfo.vehClass = obus.obu(index).vehinfodesc().vehicleclass();
        }
        temp_Bsm.bsmID = obus.obu(index).localid();
        temp_Bsm.deviceID = obus.obu(index).deviceid();
        g_BsmList.push_back(temp_Bsm);
    }
    this->v2xReceivedFlag = true;
    for (std::vector<v2x_preprocessing::BSM>::iterator it = this->g_BsmList.begin(); it !=this->g_BsmList.end();it++){
            // std::cout<<"\n*******************\n"<< getTime()<< ": Get v2x from OBU: bsmId: " << it->bsmID << std::endl;
            std::cout<<"\n*******************\n" << ": Get v2x from OBU: bsmId: " << it->bsmID << std::endl;
    }
}

void DFClient::reqGetHostVehInfo(void){
   this->invoke(NLService::REQ_GET_OBUHOSTINFO);
   this->onReplyTirgger = false;
}

void DFClient::getHostVehfromOBU(const NLService::OBUHostInfo &hvInfo){

    if (hvInfo.gnss().has_timebase()){
            g_hostVehInfo.timeStamp = hvInfo.gnss().timebase().miilsecond(); 
    }
    if (hvInfo.gnss().has_point()){
        g_hostVehInfo.hvPosition.longitude = hvInfo.gnss().point().longitude(); 
        g_hostVehInfo.hvPosition.latitude = hvInfo.gnss().point().latitude();
        g_hostVehInfo.hvPosition.positioningState = v2x_preprocessing::Pos::PositioningState(hvInfo.gnss().positioningstate()); 
    }
    if (hvInfo.has_state() && hvInfo.state().speedcanvalid() == 1) {
        g_hostVehInfo.hvSpeed = hvInfo.state().speedcan(); 
    } else{
        g_hostVehInfo.hvSpeed = hvInfo.gnss().speed(); 
    }

    if (hvInfo.has_state() && hvInfo.state().headingcanvalid() == 1) {
        g_hostVehInfo.hvSpeed = hvInfo.state().headingcan(); 
    } else{
        g_hostVehInfo.hvSpeed = hvInfo.gnss().heading(); 
    }
    g_hostVehInfo.hvAccel = hvInfo.gnss().accel();

    // std::cout << "***********************\n" << getTime() << ": Get Host Vehicle Data, hvPosition:  " << this->g_hostVehInfo.hvPosition.latitude <<", " << this->g_hostVehInfo.hvPosition.longitude << std::endl;
    // std::cout << getTime() << ": Get Host Vehicle Data, timestamp:  " << this->g_hostVehInfo.timeStamp << "\n*********" <<std::endl;
    
    std::cout << "***********************\n"  << ": Get Host Vehicle Data, hvPosition:  " << this->g_hostVehInfo.hvPosition.latitude <<", " << this->g_hostVehInfo.hvPosition.longitude << std::endl;
    std::cout << ": Get Host Vehicle Data, timestamp:  " << this->g_hostVehInfo.timeStamp << "\n*********" <<std::endl;

}

void DFClient::sendFusionData(const data_fusion::FusionObjects& fusionData)
{

    // create fusion data object
    v2x_six_sense_fusion::FusionVehicles fusionVehList;
    int vehSize = fusionData.totalCount;

    // set value
    fusionVehList.set_totalcount(fusionData.totalCount);
    for (int index = 0; index < vehSize; index++){
        v2x_six_sense_fusion::FusionVehicle *new_fusionvehicle = fusionVehList.add_fusionvehicles();

        new_fusionvehicle->set_v2xid(fusionData.fusionVehicles[index].v2xId);
        new_fusionvehicle->set_id(fusionData.fusionVehicles[index].id);
        new_fusionvehicle->set_timestamp(fusionData.fusionVehicles[index].timestamp);

        // convert v2x_preprocessing::RelativePos to v2x_six_sense_fusion::RelativePos
        
        v2x_six_sense_fusion::RelativePos new_relativePos;
        new_relativePos.set_x(fusionData.fusionVehicles[index].relativePos.x);
        new_relativePos.set_y(fusionData.fusionVehicles[index].relativePos.y);
        new_relativePos.set_z(fusionData.fusionVehicles[index].relativePos.z);
        new_fusionvehicle->mutable_relativepos()->CopyFrom(new_relativePos);
        
         // convert v2x_preprocessing::RelativeSpeed to v2x_six_sense_fusion::RelativeSpeed
        v2x_six_sense_fusion::RelativeSpeed new_relativeSpeed;
        //new_relativeSpeed.set_speedval(fusionData.fusionVehicles[index].relativeSpeed.speedVal);
        new_relativeSpeed.set_speed_x(fusionData.fusionVehicles[index].relativeSpeed.speedValx);
        new_relativeSpeed.set_speed_y(fusionData.fusionVehicles[index].relativeSpeed.speedValy);
        new_fusionvehicle->mutable_relativespeed()->CopyFrom(new_relativeSpeed);

        // convert v2x_preprocessing::VehSize to v2x_six_sense_fusion::VehSize
        v2x_six_sense_fusion::VehSize new_vehSize;
        new_vehSize.set_width(fusionData.fusionVehicles[index].vehSize.width);
        new_vehSize.set_length(fusionData.fusionVehicles[index].vehSize.length);
        new_vehSize.set_height(fusionData.fusionVehicles[index].vehSize.height);
        new_fusionvehicle->mutable_vehsize()->CopyFrom(new_vehSize);

        new_fusionvehicle->set_cooperationability(fusionData.fusionVehicles[index].cooperationAbility);
        
        // convert v2x_preprocessing::heangding to v2x_six_sense_fusion::heading
        v2x_six_sense_fusion::Heading new_heading;
        new_heading.set_headingval(fusionData.fusionVehicles[index].heading.headingVal);
        new_fusionvehicle->mutable_heading()->CopyFrom(new_heading);

        // convert v2x_preprocessing::ObjectType to v2x_six_sense_fusion::ObjectType
        v2x_six_sense_fusion::ObjectType new_objType;
        switch (fusionData.fusionVehicles[index].objType){
            case data_fusion::ObjectType::type_unknown:
                new_objType = v2x_six_sense_fusion::type_unknown;
                break;
            case data_fusion::ObjectType::vehicle:
                new_objType = v2x_six_sense_fusion::vehicle;
                break;
            default:
                new_objType = v2x_six_sense_fusion::type_unknown;
                break;
        }
        new_fusionvehicle->set_objtype(new_objType);

        // convert v2x_preprocessing::Datasource to v2x_six_sense_fusion::Datasource
        v2x_six_sense_fusion::Datasource new_datasource;
        switch (fusionData.fusionVehicles[index].datasource){
            case data_fusion::Datasource::source_unknown:
                new_datasource = v2x_six_sense_fusion::source_unknown;
                break;
            case data_fusion::Datasource::fusion:
                new_datasource = v2x_six_sense_fusion::fusion;
                break;
            case data_fusion::Datasource::sensor_only:
                new_datasource = v2x_six_sense_fusion::sensor_only;
                break;
            case data_fusion::Datasource::v2x_BSM_only:
                new_datasource = v2x_six_sense_fusion::v2x_BSM_only;
                break;
            case data_fusion::Datasource::v2x_RSM_only:
                new_datasource = v2x_six_sense_fusion::v2x_RSM_only;
                break;
            default:
                new_datasource = v2x_six_sense_fusion::source_unknown;
                break;
        }
        new_fusionvehicle->set_datasource(new_datasource);

        // convert v2x_preprocessing::GlobalPos to v2x_six_sense_fusion::GlobalPos
        v2x_six_sense_fusion::GlobalPos new_globalPos;
        new_globalPos.set_latitude(fusionData.fusionVehicles[index].globalPos.latitude);
        new_globalPos.set_longitude(fusionData.fusionVehicles[index].globalPos.longitude);
        new_globalPos.set_wgs84(fusionData.fusionVehicles[index].globalPos.WGS84);
        new_fusionvehicle->mutable_globalpos()->CopyFrom(new_globalPos);

        // convert v2x_preprocessing::AbsoluteSpeed to v2x_six_sense_fusion::AbsoluteSpeed
        v2x_six_sense_fusion::AbsoluteSpeed new_absoluteSpeed;
        new_absoluteSpeed.set_speedval(fusionData.fusionVehicles[index].absoluteSpeed.speedValx);
        new_fusionvehicle->mutable_absolutespeed()->CopyFrom(new_absoluteSpeed);
    }

    // create a protobuf msg builder object and serialization
    CFdbProtoMsgBuilder builder(fusionVehList);

    // send to server
	this->invoke(NLService::REQ_SET_FILTERED_VEHS, builder);
    std::cout << "send fusion data info out, object 1 id: " << this->_fusionData.fusionVehicles[0].id << std::endl;

}



void DFClient::sendLaneInfo(const line_v2::Lines& linesData){
    // send lane data
    // Add judgement: if no Input, do not send

    NLLane::Lines newLanes;

    // copy the header, dtlc, ttlc, src_time_stamp, asyn_time_stamp, tag and pitch_info fields
    newLanes.mutable_header()->CopyFrom(linesData.header());
    newLanes.set_dtlc(linesData.dtlc());
    newLanes.set_ttlc(linesData.ttlc());
    newLanes.set_src_time_stamp(linesData.src_time_stamp());
    newLanes.set_asyn_time_stamp(linesData.asyn_time_stamp());
    newLanes.set_tag(linesData.tag());

    NLLane::PitchInfo new_pitchInfo;
    new_pitchInfo.set_valid(linesData.pitch_info().valid());
    new_pitchInfo.set_pitch((linesData.pitch_info().pitch()));
    newLanes.mutable_pitch_info()->CopyFrom(new_pitchInfo);

    // copy the selected fields of each line
    for (int i = 0; i < linesData.lines_size(); i++)
    {
        // create a new line object
        NLLane::Line *new_line = newLanes.add_lines();

        // copy the id, str_id, life_time, type, source, position and conf fields
        new_line->set_id(linesData.lines(i).id());
        new_line->set_str_id(linesData.lines(i).str_id());
        new_line->set_life_time(linesData.lines(i).life_time());
        new_line->set_type(linesData.lines(i).type());
        new_line->set_source(linesData.lines(i).source());
        new_line->set_position(linesData.lines(i).position());
        new_line->set_conf(linesData.lines(i).conf());


        // copy the selected fields of each curve line
        for (int j = 0; j < linesData.lines(i).lines_3d_size(); j++)
        {
            // create a new curve line object
            NLLane::CurveLine *new_curve_line = new_line->add_lines_3d();

            // copy the pred_point, succ_point, width, type, start_pt, x_coeff, y_coeff, z_coeff, t_max, color, marking, parsing_conf, rmse and visible fields
            new_curve_line->mutable_pred_point()->CopyFrom(linesData.lines(i).lines_3d(j).pred_point());
            new_curve_line->mutable_succ_point()->CopyFrom(linesData.lines(i).lines_3d(j).succ_point());
            new_curve_line->set_width(linesData.lines(i).lines_3d(j).width());
            new_curve_line->set_type(linesData.lines(i).lines_3d(j).type());
            new_curve_line->mutable_start_pt()->CopyFrom(linesData.lines(i).lines_3d(j).start_pt());
            // for (int k = 0; k < linesData.lines(i).lines_3d(j).x_coeff_size(); k++)
            // {
            //     new_curve_line->add_x_coeff(linesData.lines(i).lines_3d(j).x_coeff(k));
            // }
            for (int k = 0; k < linesData.lines(i).lines_3d(j).y_coeff_size(); k++)
            {
                new_curve_line->add_y_coeff(linesData.lines(i).lines_3d(j).y_coeff(k));
            }
            // for (int k = 0; k < linesData.lines(i).lines_3d(j).z_coeff_size(); k++)
            // {
            //     new_curve_line->add_z_coeff(linesData.lines(i).lines_3d(j).z_coeff(k));
            // }
            new_curve_line->set_t_max(linesData.lines(i).lines_3d(j).t_max());
            new_curve_line->set_color(linesData.lines(i).lines_3d(j).color());
            new_curve_line->set_marking(linesData.lines(i).lines_3d(j).marking());
            new_curve_line->set_parsing_conf(linesData.lines(i).lines_3d(j).parsing_conf());
            new_curve_line->set_rmse(linesData.lines(i).lines_3d(j).rmse());
        }
    }

    // create a protobuf msg builder object and serialization
    CFdbProtoMsgBuilder builder(newLanes);

    // send to server
    this->invoke(NLService::REQ_SET_LANE_INFO, builder);
    std::cout << "Client Send lane info: " << newLanes.src_time_stamp() << std::endl;

    // std::cout << "obu Client send lane info: \n" << newLanes.DebugString() <<"\n*********** \n"<< std::endl;


    
}

bool DFClient::BsmUpdate() {

    

    return true;
    
}

void DFClient::waitForOnReplyTrigger(){
    std::unique_lock<std::mutex> lock(onReplyMtx);
    if (!onReplyTirgger){
        onReplyCv.wait(lock,[this]{
            return onReplyTirgger;});
    }
}

void DFClient::setOnReplyTrigger(){
    std::lock_guard<std::mutex> lock(onReplyMtx);
    onReplyTirgger = true;
    onReplyCv.notify_all();
}
