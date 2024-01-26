#ifndef MSGCC
#define MSGCC

#define TCP_DATA_HEAD_SIZE 1
#define TCP_DATA_LENGTH_SIZE_CC 4
#define TCP_DATA_STR_LENGTH_CC 16
#define TCP_IO_NUMBER_CC 16
#define TCP_ANALOG_NUMBER_CC 8
#include <string>
#include <cstring>

#ifdef __cplusplus
extern "C"
{
#endif

    namespace MSG_CC
    {
    struct BaseStruct
    {
        unsigned char m_dataLength[TCP_DATA_LENGTH_SIZE_CC];
        unsigned char m_head;
        unsigned int m_jobId;
        int m_agvId;

        BaseStruct()
        {
            int s = sizeof(BaseStruct);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            m_head = 0x00;
            m_jobId = 0;
            m_agvId = -1;
        }
        BaseStruct(BaseStruct *ptr)
        {
            memcpy(m_dataLength, ptr->m_dataLength, TCP_DATA_LENGTH_SIZE_CC);
            m_head = ptr->m_head;
            m_jobId = ptr->m_jobId;
            m_agvId = ptr->m_agvId;
        }
        unsigned char GetType(void) { return m_head; }
    };

    enum DataHeadEnum
    {
        _moveHeadEnum = 0x00,
        _grabmapHeadEnum = 0x01,
        _lineHeadEnum = 0x02,
        _excutmacroHeadEnum = 0x03,
        _setmodeHeadEnum = 0x04,
        _errdelHeadEnum = 0x05,
        _ctlHeadEnum = 0x06,
        _uplaserHeadEnum = 0x07,
        _relocHeadEnum = 0x08,
        _cnlwaitHeadEnum = 0x09, //
        _setioHeadEnum = 0x0A,
        _setanaHeadEnum = 0x0B,
        _arearesponseHeadEnum = 0x0C,    //
        _agvneighborListHeadEnum = 0x0D, //
        _agvHeadEnum = 0x0E,             //
        _askagvcurrentmapHeadEnum = 0x0F,
        _heartHeadEnum = 0x10,
        _stopHeadEnum = 0x11,
        _safelaserHeadEnum = 0x12,
        _emergencyStopHeadEnum = 0x13,
        _setworkmodeHeadEnum = 0x14,         // 设置AGV工作模式：手动、半自动、全自动三种模式
        _openSlamHeadEnum = 0x15,            // 打开建图开关
        _agvRouteHeadEnum = 0x22,
        _answerConfigHeadEnum = 0x24,
        _agvReadLocalConfigHeadEnum = 0x25,  // Deploy发给agv，告知agv本地的配置文件有变，叫agv重新读入
        _restartAgvCmdHeadEnum = 0x26,       // Deploy或者CC发送给AGV，叫AGV重启
        _stopAgvChargeHeadEnum = 0x27,       // agv停止充电
        _launchSourceHeadEnum = 0x28,        // CC向agv发起开始对齐资源
        _broadcastAgvHeadEnum = 0x29,        // CC向agv广播其领域agv
        _answerTimeHeadEnum = 0x2A,          // CC回复小车时间
        _gotoGoalHeadEnum = 0x2C,            // 只到达目标点，不执行目标点下的macro
        _answerMapVersionHeadEnum = 0x30,    // CC回复小车地图的版本号
        _gotoChargeHeadEnum = 0xF8,          // 充电指令
        _resourceRecoverHeadEnum = 0xF9,     // 资源对齐
        _clearAllJobsOnAgvHeadEnum = 0x31,   // 清除所有任务
        _screenDisplayHeadEnum = 0x32,       // 要求小车屏幕上显示指定的字符串，变长消息，没有具体的结构体
        _agvNewRouteHeadEnum   = 0x33,      // AGV的新路线格式
        _followPointHeadEnum = 0x40,                 // 跟随任务中的单个点
        _followJobFinishHeadEnum = 0x41,             // 跟随任务结束
    };

    //心跳包
    struct HeartBeat : public BaseStruct
    {
        HeartBeat()
        {
            m_head = _heartHeadEnum;
            int s = sizeof(HeartBeat);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        HeartBeat(HeartBeat *ptr) : BaseStruct((BaseStruct *)ptr) {}
        explicit HeartBeat(HeartBeat &ptr) : BaseStruct(&ptr) {}
    };

    //Jobs
    struct MoveJob : public BaseStruct
    {
        /**
       * Move to goal name
       * Move to anywhere x,y,theta
       */
        enum JobType
        {
            MoveToGoal = 0x00,
            MoveToPoint = 0x01
        };
        MoveJob() : m_x(0), m_y(0), m_theta(0.0), m_jobtype(MoveToGoal)
        {
            m_head = _moveHeadEnum;
            int s = sizeof(MoveJob);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            memset(m_goalName, '\0', TCP_DATA_STR_LENGTH_CC);
            memset(m_mapFile, '\0', TCP_DATA_STR_LENGTH_CC);
        }
        MoveJob(MoveJob *ptr) : BaseStruct((BaseStruct *)ptr),
                                m_x(ptr->m_x), m_y(ptr->m_y), m_theta(ptr->m_theta), m_jobtype(ptr->m_jobtype)
        {
            memcpy(m_goalName, ptr->m_goalName, TCP_DATA_STR_LENGTH_CC);
            memcpy(m_mapFile, ptr->m_mapFile, TCP_DATA_STR_LENGTH_CC);
        }
        MoveJob(MoveJob &ptr) : BaseStruct(&ptr),
                                m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta), m_jobtype(ptr.m_jobtype)
        {
            memcpy(m_goalName, ptr.m_goalName, TCP_DATA_STR_LENGTH_CC);
            memcpy(m_mapFile, ptr.m_mapFile, TCP_DATA_STR_LENGTH_CC);
        }
        unsigned char m_jobtype;
        unsigned char m_goalName[TCP_DATA_STR_LENGTH_CC];
        int m_x, m_y; //毫米
        float m_theta;
        unsigned char m_mapFile[TCP_DATA_STR_LENGTH_CC];
    };

    struct FollowPoint : public BaseStruct{
        FollowPoint() : m_x(0), m_y(0), m_theta(0.0)
        {
            m_head = _followPointHeadEnum;
            int s = sizeof(FollowPoint);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        FollowPoint(FollowPoint *ptr) : BaseStruct((BaseStruct *)ptr),
                                m_x(ptr->m_x), m_y(ptr->m_y), m_theta(ptr->m_theta),
                                m_time(ptr->m_time)
        {
        }
        FollowPoint(FollowPoint &ptr) : BaseStruct(&ptr),
                                m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta), 
                                m_time(ptr.m_time)
        {
        }
        float m_x, m_y, m_theta;
        long m_time;
    };

    // 正常结束, 当前任务完成
    struct FollowJobFinish: public BaseStruct{
        FollowJobFinish(){
            m_head = _followJobFinishHeadEnum;
            int s = sizeof(FollowJobFinish);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }

        FollowJobFinish(FollowJobFinish *ptr): BaseStruct((BaseStruct *)ptr){}
        FollowJobFinish(FollowJobFinish &ptr): BaseStruct(&ptr){}
    };

    // 只到达目标工位，不执行工位下的macro
    struct GotoGoalJob : public BaseStruct
    {
        //_gotoGoalHeadEnum
        GotoGoalJob(){
            m_head = _gotoGoalHeadEnum;//0x2C
            int s = sizeof(GotoGoalJob);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            memset(m_goalName, '\0', TCP_DATA_STR_LENGTH_CC);
            memset(m_mapfile, '\0', TCP_DATA_STR_LENGTH_CC);
        }
        GotoGoalJob(GotoGoalJob* ptr) : BaseStruct((BaseStruct*)ptr){
            memcpy(m_goalName, ptr->m_goalName, TCP_DATA_STR_LENGTH_CC);
            memcpy(m_mapfile, ptr->m_mapfile, TCP_DATA_STR_LENGTH_CC);
        }
        explicit GotoGoalJob(GotoGoalJob& ptr) : BaseStruct(&ptr) {
            memcpy(m_goalName, ptr.m_goalName, TCP_DATA_STR_LENGTH_CC);
            memcpy(m_mapfile, ptr.m_mapfile, TCP_DATA_STR_LENGTH_CC); 
        }
        unsigned char m_goalName[TCP_DATA_STR_LENGTH_CC];
        unsigned char m_mapfile[TCP_DATA_STR_LENGTH_CC];
    };

    struct GrabMapJob : public BaseStruct
    {
        /**
       * Grap map
       */
        GrabMapJob()
        {
            m_head = _grabmapHeadEnum;
            int s = sizeof(GrabMapJob);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            memset(m_mapFileName, '\0', TCP_DATA_STR_LENGTH_CC);
        }
        GrabMapJob(GrabMapJob *ptr) : BaseStruct((BaseStruct *)ptr)
        {
            memcpy(m_mapFileName, ptr->m_mapFileName, TCP_DATA_STR_LENGTH_CC);
        }
        GrabMapJob(GrabMapJob &ptr) : BaseStruct(&ptr)
        {
            memcpy(m_mapFileName, ptr.m_mapFileName, TCP_DATA_STR_LENGTH_CC);
        }
        unsigned char m_mapFileName[TCP_DATA_STR_LENGTH_CC];
    };

    struct ExecuteMacroJob : public BaseStruct
    {
        /**
       * Execute macro
       */
        ExecuteMacroJob()
        {
            m_head = _excutmacroHeadEnum;
            int s = sizeof(ExecuteMacroJob);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            memset(m_macro, '\0', TCP_DATA_STR_LENGTH_CC);
        }
        ExecuteMacroJob(ExecuteMacroJob *ptr) : BaseStruct((BaseStruct *)ptr)
        {
            memcpy(m_macro, ptr->m_macro, TCP_DATA_STR_LENGTH_CC);
        }
        ExecuteMacroJob(ExecuteMacroJob &ptr) : BaseStruct(&ptr)
        {
            memcpy(m_macro, ptr.m_macro, TCP_DATA_STR_LENGTH_CC);
        }
        unsigned char m_macro[TCP_DATA_STR_LENGTH_CC];
    };

    //Command, 不需要等待agv回复
    //不会因为下发这个消息而让agv变成忙碌状态
    //但需要agv回复消息，是否成功，决定了agv的状态
    //下发的时候不需要考虑agv是否忙碌
    struct SetModeCommand : public BaseStruct
    {
        /**
       * Set agv mode
       */
        enum agvWorkModeEnum
        {
            WorkMode = 0x00, //正常可工作模式
            SlamMode = 0x01, //手动建图模式
            EditMode = 0x02, //手动编辑模式
            MoveMode = 0x03  //手动可移动模式
        };
        SetModeCommand() : m_mode(WorkMode)
        {
            m_head = _setmodeHeadEnum;
            int s = sizeof(SetModeCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        SetModeCommand(SetModeCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_mode(ptr->m_mode) {}
        SetModeCommand(SetModeCommand &ptr) : BaseStruct(&ptr), m_mode(ptr.m_mode) {}
        unsigned char m_mode;
    };

    //打开/关闭安全激光
    struct SafeLaserCommand : public BaseStruct
    {
        enum SafeLaserModeEnum
        {
            OnMode = 0x00, //打开
            OffMode = 0x01 //关闭
        };
        SafeLaserCommand() : m_mode(OnMode)
        {
            m_head = _safelaserHeadEnum;
            int s = sizeof(SafeLaserCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        SafeLaserCommand(SafeLaserCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_mode(ptr->m_mode) {}
        SafeLaserCommand(SafeLaserCommand &ptr) : BaseStruct(&ptr), m_mode(ptr.m_mode) {}
        unsigned char m_mode;
    };

    struct OnOfflineCommand : public BaseStruct
    {
        /**
     * On line / offline agv
     */
        enum LineMode
        {
            OnlineMode = 0x00,
            OfflineMode = 0x01
        };
        OnOfflineCommand() : m_onOffLine(OfflineMode)
        {
            m_head = _lineHeadEnum;
            int s = sizeof(OnOfflineCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        OnOfflineCommand(OnOfflineCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_onOffLine(ptr->m_onOffLine) {}
        OnOfflineCommand(OnOfflineCommand &ptr) : BaseStruct(&ptr), m_onOffLine(ptr.m_onOffLine) {}
        unsigned char m_onOffLine;
    };

    struct ErrorDelCommand : public BaseStruct
    {
        ErrorDelCommand() : m_errorNum(0)
        {
            m_head = _errdelHeadEnum;
            int s = sizeof(ErrorDelCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        ErrorDelCommand(ErrorDelCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_errorNum(ptr->m_errorNum) {}
        ErrorDelCommand(ErrorDelCommand &ptr) : BaseStruct(&ptr), m_errorNum(ptr.m_errorNum) {}
        unsigned char m_errorNum;
    };

    struct ControlCommand : public BaseStruct
    {
        /**
     * Cancel Job    取消任务
     * Continue Job  继续任务  用于出现中断异常并解决了该异常后，继续执行任务
     */
        enum CtlType
        {
            CancelCtlType = 0x00,
            ContinueCtlType = 0x01
        };
        ControlCommand() : m_control(0)
        {
            m_head = _ctlHeadEnum;
            int s = sizeof(ControlCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        ControlCommand(ControlCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_control(ptr->m_control) {}
        ControlCommand(ControlCommand &ptr) : BaseStruct(&ptr), m_control(ptr.m_control) {}
        unsigned char m_control;
    };

    struct AskAgvCurrentCommand : public BaseStruct
    {
        /*		Ask agv current map filename		*/
        AskAgvCurrentCommand()
        {
            m_head = _askagvcurrentmapHeadEnum;
            int s = sizeof(AskAgvCurrentCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        AskAgvCurrentCommand(AskAgvCurrentCommand *ptr) : BaseStruct((BaseStruct *)ptr) {}
        AskAgvCurrentCommand(AskAgvCurrentCommand &ptr) : BaseStruct(&ptr) {}
    };

    struct UploadLaserCommand : public BaseStruct
    {
        /* Upload laser */
        enum switchEnum
        {
            switchOnEnum = 0x00,
            switchOffEnum = 0x01
        };
        UploadLaserCommand() : m_switch(0)
        {
            m_head = _uplaserHeadEnum;
            int s = sizeof(UploadLaserCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        UploadLaserCommand(UploadLaserCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_switch(ptr->m_switch) {}
        UploadLaserCommand(UploadLaserCommand &ptr) : BaseStruct(&ptr), m_switch(ptr.m_switch) {}
        unsigned char m_switch;
    };

    struct RelocCommand : public BaseStruct
    {
        /* Reloc */
        RelocCommand() : m_x(0), m_y(0), m_theta(0)
        {
            m_head = _relocHeadEnum;
            int s = sizeof(RelocCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        RelocCommand(RelocCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_x(ptr->m_x), m_y(ptr->m_y), m_theta(ptr->m_theta) {}
        RelocCommand(RelocCommand &ptr) : BaseStruct(&ptr), m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta) {}
        float m_x, m_y, m_theta;
    };

    struct CancelWaitCommand : public BaseStruct
    {
        /* Cancel wait forever */
        CancelWaitCommand()
        {
            m_head = _cnlwaitHeadEnum;
            int s = sizeof(CancelWaitCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        CancelWaitCommand(CancelWaitCommand *ptr) : BaseStruct((BaseStruct *)ptr) {}
        CancelWaitCommand(CancelWaitCommand &ptr) : BaseStruct(&ptr) {}
    };

    struct SetIoCommand : public BaseStruct
    {
        /* Set IO data */
        SetIoCommand()
        {
            m_head = _setioHeadEnum;
            int s = sizeof(SetIoCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            m_io = 0;
            m_idx = 0;
        }
        SetIoCommand(SetIoCommand *ptr) : BaseStruct((BaseStruct *)ptr)
        {
            m_io = ptr->m_io;
            m_idx = ptr->m_idx;
        }
        SetIoCommand(SetIoCommand &ptr) : BaseStruct(&ptr)
        {
            m_io = ptr.m_io;
            m_idx = ptr.m_idx;
        }
        unsigned char m_io;
        unsigned char m_idx;
    };

    struct SetAnaCommand : public BaseStruct
    {
        /*
		Set analog data
		*/
        SetAnaCommand()
        {
            m_head = _setanaHeadEnum;
            int s = sizeof(SetAnaCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            m_ana = 0.0;
            m_idx = 0;
        }
        SetAnaCommand(SetAnaCommand *ptr) : BaseStruct((BaseStruct *)ptr)
        {
            m_ana = ptr->m_ana;
            m_idx = ptr->m_idx;
        }
        SetAnaCommand(SetAnaCommand &ptr) : BaseStruct(&ptr)
        {
            m_ana = ptr.m_ana;
            m_idx = ptr.m_idx;
        }
        float m_ana;
        unsigned char m_idx;
    };

    //stop agv
    struct StopAgvCommand : public BaseStruct
    {
        /*
		Set agv mode
		*/
        enum agvMoveModeEnum
        {
            StopMode = 0x00,
            RestartMode = 0x01
        };

        StopAgvCommand() : m_mode(StopMode)
        {
            m_head = _stopHeadEnum;
            int s = sizeof(StopAgvCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        StopAgvCommand(StopAgvCommand *ptr) : BaseStruct((BaseStruct *)ptr), m_mode(ptr->m_mode) {}
        StopAgvCommand(StopAgvCommand &ptr) : BaseStruct(&ptr), m_mode(ptr.m_mode) {}
        unsigned char m_mode;
    };

    // Response to agv
    struct AreaRequestResponse : public BaseStruct
    {
        /*		*/
        enum SourceType
        {
            AreaSourceType = 0x00,
            GoalSourceType = 0x01
        };
        enum ResponseEnum
        {
            RefuseEnum = 0x00,
            AcceptEnum = 0x01
        };
        AreaRequestResponse() : m_response(RefuseEnum)
        {
            m_head = _arearesponseHeadEnum;
            int s = sizeof(AreaRequestResponse);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            memset(m_area, '\0', TCP_DATA_STR_LENGTH_CC);
            memset(m_standby, '\0', TCP_DATA_STR_LENGTH_CC);
            m_source_type = AreaSourceType;
        }
        AreaRequestResponse(AreaRequestResponse *ptr) : BaseStruct((BaseStruct *)ptr),
                                                        m_response(ptr->m_response), m_source_type(ptr->m_source_type)
        {
            memcpy(m_area, ptr->m_area, TCP_DATA_STR_LENGTH_CC);
            memcpy(m_standby, ptr->m_standby, TCP_DATA_STR_LENGTH_CC);
        }
        AreaRequestResponse(AreaRequestResponse &ptr) : BaseStruct(&ptr),
                                                        m_response(ptr.m_response), m_source_type(ptr.m_source_type)
        {
            memcpy(m_area, ptr.m_area, TCP_DATA_STR_LENGTH_CC);
            memcpy(m_standby, ptr.m_standby, TCP_DATA_STR_LENGTH_CC);
        }
        unsigned char m_response;
        unsigned char m_area[TCP_DATA_STR_LENGTH_CC];
        unsigned char m_standby[TCP_DATA_STR_LENGTH_CC];
        unsigned char m_source_type;
    };

    struct OneAgvNeighborResponse : public BaseStruct
    {
        /*
		这只是一个agv的信息，实际下发的时候会打包成多个agv的消息，消息变长
		*/
        OneAgvNeighborResponse() : m_x(0.0), m_y(0.0), m_theta(0.0), m_width(0.0), m_height(0.0)
        {
            m_head = _agvHeadEnum;
            int s = sizeof(OneAgvNeighborResponse);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        OneAgvNeighborResponse(OneAgvNeighborResponse *ptr) : BaseStruct((BaseStruct *)ptr),
                                                              m_x(ptr->m_x), m_y(ptr->m_y), m_theta(ptr->m_theta), m_width(ptr->m_width), m_height(ptr->m_height) {}
        OneAgvNeighborResponse(OneAgvNeighborResponse &ptr) : BaseStruct(&ptr),
                                                              m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta), m_width(ptr.m_width), m_height(ptr.m_height) {}

        float m_x, m_y, m_theta;
        unsigned char m_width, m_height;
    };

    struct AgvRoute
    {
        AgvRoute()
        {
            m_head = _agvRouteHeadEnum;
            int s = sizeof(AgvRoute);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            m_agvId = -1;
            m_routeLength = 0;
            m_jobId = 0;
        }
        AgvRoute(AgvRoute *ptr) : m_agvId(ptr->m_agvId), m_routeLength(ptr->m_routeLength), m_jobId(ptr->m_jobId)
        {
            m_head = _agvRouteHeadEnum;
            int s = sizeof(AgvRoute);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        AgvRoute(AgvRoute &ptr) : m_agvId(ptr.m_agvId), m_routeLength(ptr.m_routeLength), m_jobId(ptr.m_jobId)
        {
            m_head = _agvRouteHeadEnum;
            int s = sizeof(AgvRoute);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        unsigned char m_dataLength[TCP_DATA_LENGTH_SIZE_CC];
        unsigned char m_head;
        int m_agvId;
        int m_routeLength;
        int m_jobId;
    };

    struct AgvRouteOnePoint
    {
        AgvRouteOnePoint() {}
        AgvRouteOnePoint(float _x, float _y, float _t, float c) : m_x(_x), m_y(_y), m_theta(_t), m_cur(c) {}
        AgvRouteOnePoint(AgvRouteOnePoint *ptr) : m_x(ptr->m_x), m_y(ptr->m_y), m_theta(ptr->m_theta), m_cur(ptr->m_cur) {}
        AgvRouteOnePoint(const AgvRouteOnePoint &ptr) : m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta), m_cur(ptr.m_cur) {}

        AgvRouteOnePoint &operator=(const AgvRouteOnePoint &p)
        {
            if (this != &p)
            {
                this->m_x = p.m_x;
                this->m_y = p.m_y;
                this->m_theta = p.m_theta;
                this->m_cur = p.m_cur;
            }
            return *this;
        }

        float m_x, m_y, m_theta, m_cur;
    };

    struct AnswerConfig : public BaseStruct
    {
        AnswerConfig()
        {
            m_head = _answerConfigHeadEnum;
            int s = sizeof(AnswerConfig);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            memset(m_path, '\0', TCP_DATA_STR_LENGTH_CC);
        }
        AnswerConfig(AnswerConfig *ptr) : BaseStruct((BaseStruct *)ptr) { memcpy(m_path, ptr->m_path, TCP_DATA_STR_LENGTH_CC); }
        AnswerConfig(AnswerConfig &ptr) : BaseStruct(&ptr) { memcpy(m_path, ptr.m_path, TCP_DATA_STR_LENGTH_CC); }

        unsigned char m_path[TCP_DATA_STR_LENGTH_CC];
    };

    struct AgvReadLocalConfig : public BaseStruct
    {
        AgvReadLocalConfig()
        {
            m_head = _agvReadLocalConfigHeadEnum;
            int s = sizeof(AgvReadLocalConfig);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        AgvReadLocalConfig(AgvReadLocalConfig *ptr) : BaseStruct((BaseStruct *)ptr) {}
        AgvReadLocalConfig(AgvReadLocalConfig &ptr) : BaseStruct((BaseStruct &)ptr) {}
    };

    struct RestartAgvCmd : public BaseStruct
    {
        RestartAgvCmd()
        {
            int s = sizeof(RestartAgvCmd);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            m_head = _restartAgvCmdHeadEnum;
        }
        RestartAgvCmd(RestartAgvCmd *ptr) : BaseStruct((BaseStruct *)ptr) {}
        RestartAgvCmd(RestartAgvCmd &ptr) : BaseStruct((BaseStruct &)ptr) {}
    };

    //调度告知小车停止充电
    struct StopAgvChargeCmd : public BaseStruct
    {
        StopAgvChargeCmd()
        {
            int s = sizeof(StopAgvChargeCmd);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
            m_head = _stopAgvChargeHeadEnum;
        }
        StopAgvChargeCmd(StopAgvChargeCmd *ptr) : BaseStruct((BaseStruct *)ptr) {}
        StopAgvChargeCmd(StopAgvChargeCmd &ptr) : BaseStruct((BaseStruct &)ptr) {}
    };

    struct LaunchSourceCmd : public BaseStruct
	{
		LaunchSourceCmd() {
			int s = sizeof(LaunchSourceCmd);
			memcpy(m_dataLength, &s , TCP_DATA_LENGTH_SIZE_CC);
			m_head = _launchSourceHeadEnum;
		}
		LaunchSourceCmd(LaunchSourceCmd* ptr ) : BaseStruct((BaseStruct*)ptr){}
		LaunchSourceCmd(LaunchSourceCmd& ptr ) : BaseStruct((BaseStruct&)ptr){}
	};

    struct BroadCastAgv  
	{
		/*
		实车联调时，向小车广播其领域小车的信息，这个结构体只包括一个小车的信息
		*/
		BroadCastAgv( ) : m_x(0), m_y(0), m_theta(0), m_robot_front_x(0), m_robot_rear_x(0), m_robot_y(0){
 		}
		BroadCastAgv( BroadCastAgv* ptr ) : m_x ( ptr->m_x ) , m_y (ptr->m_y ) , m_theta ( ptr->m_theta),
			m_robot_front_x(ptr->m_robot_front_x) , m_robot_rear_x(ptr->m_robot_rear_x), m_robot_y(ptr->m_robot_y){
 		}
		BroadCastAgv(const BroadCastAgv& ptr) : m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta),
			m_robot_front_x(ptr.m_robot_front_x), m_robot_rear_x(ptr.m_robot_rear_x), m_robot_y(ptr.m_robot_y) {
		}
		int   m_x, m_y;
		float m_theta;
		int   m_robot_front_x, m_robot_rear_x, m_robot_y;
	};

    struct AnswerTime : public BaseStruct
	{
		unsigned char m_time[14];
		AnswerTime() {
			int s = sizeof(AnswerTime);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
			m_head = _answerTimeHeadEnum;
			memset(m_time, 0, 14);
		}
		AnswerTime(AnswerTime* ptr) : BaseStruct((BaseStruct*)ptr) {
			memcpy(m_time, ptr->m_time, 14);
		}
		AnswerTime(AnswerTime& ptr) : BaseStruct((BaseStruct&)ptr) {
			memcpy(m_time, ptr.m_time, 14);
		}
	};

    struct AnswerMapVersion : public BaseStruct
	{
		unsigned char m_mapfile[TCP_DATA_STR_LENGTH_CC];
		unsigned char m_version[14];
		AnswerMapVersion() {
			int s = sizeof(AnswerMapVersion);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
			m_head = _answerMapVersionHeadEnum;
			memset(m_mapfile, '\0', TCP_DATA_STR_LENGTH_CC);
			memset(m_version, '\0', 14);
		}
		AnswerMapVersion(const AnswerMapVersion* ptr) : BaseStruct((BaseStruct*)ptr) {
			memcpy(m_mapfile, ptr->m_mapfile, TCP_DATA_STR_LENGTH_CC);
			memcpy(m_version, ptr->m_version, 14);
		}
		AnswerMapVersion(const AnswerMapVersion& ptr) : BaseStruct((BaseStruct&)ptr) {
			memcpy(m_mapfile, ptr.m_mapfile, TCP_DATA_STR_LENGTH_CC);
			memcpy(m_version, ptr.m_version, 14);
		}
	};
    
    struct GotoChargeJob : public BaseStruct
	{
		GotoChargeJob(){
			m_head = _gotoChargeHeadEnum;
			int s = sizeof(GotoChargeJob);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
			memset(m_goalName, '\0', TCP_DATA_STR_LENGTH_CC);
			memset(m_mapfile, '\0', TCP_DATA_STR_LENGTH_CC);
		}
		GotoChargeJob(GotoChargeJob* ptr) : BaseStruct((BaseStruct*)ptr){
			memcpy(m_goalName, ptr->m_goalName, TCP_DATA_STR_LENGTH_CC);
			memcpy(m_mapfile, ptr->m_mapfile, TCP_DATA_STR_LENGTH_CC);
		}
		explicit GotoChargeJob(GotoChargeJob& ptr) : BaseStruct(&ptr){
			memcpy(m_goalName, ptr.m_goalName, TCP_DATA_STR_LENGTH_CC);
			memcpy(m_mapfile, ptr.m_mapfile, TCP_DATA_STR_LENGTH_CC);
		}
 		unsigned char m_goalName[TCP_DATA_STR_LENGTH_CC];
		unsigned char m_mapfile[TCP_DATA_STR_LENGTH_CC];
	};

    struct ResourceRecover : public BaseStruct
	{
		enum ResourceTypeEnum
		{
			ResourceTypeEnum_goal = 0x00,
			ResourceTypeEnum_area = 0x01,
			ResourceTypeEnum_job = 0x02
		};
		ResourceRecover() : m_sourceType(ResourceTypeEnum_goal) {
			m_head = _resourceRecoverHeadEnum;
			memset(m_sourceName, '\0', TCP_DATA_STR_LENGTH_CC);
			int s = sizeof(ResourceRecover);
			memcpy(m_dataLength, &s, TCP_DATA_STR_LENGTH_CC);
		} 
		ResourceRecover(ResourceRecover* ptr) : BaseStruct((BaseStruct*)ptr),m_sourceType(ptr->m_sourceType) {
			memcpy(m_sourceName, ptr->m_sourceName, TCP_DATA_STR_LENGTH_CC); 
		}
		ResourceRecover(ResourceRecover& ptr) : BaseStruct(&ptr), m_sourceType(ptr.m_sourceType) {
			memcpy(m_sourceName, ptr.m_sourceName, TCP_DATA_STR_LENGTH_CC); 
		}
		unsigned char m_sourceType;
		unsigned char m_sourceName[TCP_DATA_STR_LENGTH_CC]; 
	};

    struct ClearJobsOnAgv : public BaseStruct
	{
		ClearJobsOnAgv() {
			int s = sizeof(ClearJobsOnAgv);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
			m_head = _clearAllJobsOnAgvHeadEnum;//0x31
		}
		ClearJobsOnAgv(const ClearJobsOnAgv* ptr) : BaseStruct((BaseStruct*)ptr){}
		ClearJobsOnAgv(const ClearJobsOnAgv& ptr) : BaseStruct((BaseStruct&)ptr){}
	};

    struct EmergencyStopCommand : public BaseStruct
	{
		enum CommandTypeEnum {
			CommandTypeEnumStop = 0x00,
			CommandTypeEnumGo   = 0x01
		};
		EmergencyStopCommand():m_type(CommandTypeEnumStop){
			m_head = _emergencyStopHeadEnum;//0x13
			int s = sizeof(EmergencyStopCommand);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
		}
		EmergencyStopCommand(EmergencyStopCommand *ptr) :BaseStruct((BaseStruct*)ptr),m_type(ptr->m_type){}
		EmergencyStopCommand(EmergencyStopCommand &ptr) :BaseStruct(&ptr),m_type(ptr.m_type){}
		unsigned char m_type;
	};

    struct AgvNewRoute : public BaseStruct
	{
		int  m_blockLength;
		AgvNewRoute() {
			m_head = _agvNewRouteHeadEnum;
			int s = sizeof(AgvNewRoute);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
			m_blockLength = 0;
			m_agvId = 0;
		}
		AgvNewRoute(AgvNewRoute& ptr) : BaseStruct(&ptr),m_blockLength(ptr.m_blockLength){}
		AgvNewRoute(AgvNewRoute* ptr) : BaseStruct((BaseStruct*)ptr),m_blockLength(ptr->m_blockLength){}
	};

    struct SetWorkModeCommand : public BaseStruct
    { 
        enum agvWorkModeEnum {
            ManualControlMode = 0x00, //手动模式
            SemiAutoMode = 0x01,      //半自动模式 
            AutoMode = 0x02           //自动模式   
        };

        SetWorkModeCommand() : m_mode(ManualControlMode) {
            m_head = _setworkmodeHeadEnum; // 0x14
            int s = sizeof(SetWorkModeCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        SetWorkModeCommand(SetWorkModeCommand *ptr) : BaseStruct((BaseStruct*)ptr), m_mode(ptr->m_mode) {}
        SetWorkModeCommand(SetWorkModeCommand &ptr) : BaseStruct(&ptr), m_mode(ptr.m_mode) {}
        unsigned char m_mode;
    };

    struct OpenSlamCommand : public BaseStruct
    { 
        enum OpenSlamSwitchEnum {
            OpenSlamSwitchEnum_on,
            OpenSlamSwitchEnum_off
        };
        OpenSlamCommand() : m_switch(OpenSlamSwitchEnum_on) {
            m_head = _openSlamHeadEnum; // 0x15
            int s = sizeof(OpenSlamCommand);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_CC);
        }
        OpenSlamCommand(OpenSlamCommand *ptr) : BaseStruct((BaseStruct*)ptr), m_switch(ptr->m_switch) {}
        OpenSlamCommand(OpenSlamCommand &ptr) : BaseStruct(&ptr), m_switch(ptr.m_switch) {}
        unsigned char m_switch;
    };

	struct AgvNewRouteSize
	{
		unsigned char m_dataLength[TCP_DATA_LENGTH_SIZE_CC];
		int m_blocksize;
		AgvNewRouteSize() {
			int s = sizeof(AgvNewRouteSize);
			memcpy(m_dataLength, &s , TCP_DATA_LENGTH_SIZE_CC);
			m_blocksize = 0;
		}
		AgvNewRouteSize(AgvNewRouteSize& ptr) : m_blocksize(ptr.m_blocksize)  {}
		AgvNewRouteSize(AgvNewRouteSize* ptr) : m_blocksize(ptr->m_blocksize) {}
	};

	struct AgvNewRouteOnePoint
	{
		AgvNewRouteOnePoint() { m_x = 0.0; m_y = 0.0; }
		AgvNewRouteOnePoint(double _x , double _y) : m_x(_x ) , m_y(_y){}
		AgvNewRouteOnePoint(AgvNewRouteOnePoint* ptr) : m_x(ptr->m_x),m_y(ptr->m_y){}
		explicit AgvNewRouteOnePoint(AgvNewRouteOnePoint& ptr) : m_x(ptr.m_x ), m_y (ptr.m_y){}
		double m_x, m_y;
	};
    
    } //end namespace MSG_CC

#ifdef __cplusplus
}
#endif

#endif //MSGCC
