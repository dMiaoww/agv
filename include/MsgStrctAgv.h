#ifndef MSGAGV
#define MSGAGV

#define TCP_DATA_LENGTH_SIZE_AGV 4
#define TCP_DATA_STR_LENGTH_AGV 16
#define TCP_LASER_LENGTH 1084
#include <string>
#include <cstring>

#ifdef __cplusplus
extern "C"
{
#endif
  
enum class AgvWorkMode {
  _ManualControlMode = 0x00,  // 手动模式，响应手柄控制
  _SemiAutoMode = 0x01,       // 半自动模式，响应deploy发送过来的指令
  _AutoMode = 0x02            // 自动模式，响应MES和MQTT发送过来的指令
};

enum class slamSwitch {
  _OpenSlamSwitchEnum_on = 0x00,  // 打开建图模式
  _OpenSlamSwitchEnum_off = 0x01  // 关闭建图模式
};

namespace MSG_AGV
{
    struct BaseData
    {
        unsigned char m_dataLength[TCP_DATA_LENGTH_SIZE_AGV];
        unsigned char m_head;

        BaseData()
        {
            int s = sizeof(BaseData);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = 0x00;
        }
        BaseData(BaseData *ptr)
        {
            memcpy(m_dataLength, ptr->m_dataLength, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = ptr->m_head;
        }
        unsigned char GetType(void) { return m_head; }
    };

    enum DataHeadEnum
    {
        _agvStatusHeadEnum = 0xA0,
        _agvRemainTimeHeadEnum = 0xA1,
        _agvTaskStatusHeadEnum = 0xA2,
        _agvSyncMapHeadEnum = 0xA3,
        _agvAreaHeadEnum = 0xA4,
        _agvLaserInfoHeadEnum = 0xA5,
        _avgErrDelHeadEnum = 0xA6,
        _agvErrReptHeadEnum = 0xA7,
        _agvSetModeHeadEnum = 0xA8,
        _agvSetOnlineEnum = 0xA9,
        _agvCurrentMapEnum = 0xAA,
        _agv_reject_connect = 0xAB,
        _agvSwitchMapEnum = 0xAC,
        _agvTaskResultHeadEnum = 0xAF,
        _agvAskConfigHeadEnum = 0xB0,
        _agvAttributeHeadEnum  = 0xB1, 
        _agvAskTimeHeadEnum    = 0xB2,
        _agvAskMapVersionHeadEnum = 0xB3,

        _agvRecoverHeadEnum = 0xD0,
        _agvRecoverSourceHeadEnum = 0xD1,
        _agvStandbyCannotReachHeadEnum = 0xB4,
        _agvRestartHeadEnum = 0xB5,
        _agvArriveHeadEnum = 0xB6,
        _agvLastJobStatusHeadEnum = 0xB7,
        _agvGetWrongIDTaskHeadEnum = 0xB8,      // agv接收到的任务中id不对
        _openSlamResultHeadEnum = 0xB9,         // 
        _agvSetModeResultHeadEnum = 0xBA,
        _identifiedHeadEnum = 0xD2,              // 回复AGV的身份
        _deployRequestMsgHeadEnum = 0xC0,        // deploy询问身份
        _agvFinishReadLocalConfigHeadEnum = 0xBB
    };

    //agv状态信息，任务无关
    struct AgvStatus : public BaseData
    {
        enum AttributeBitEnum {
			AttributeBitEnum_safeLaser = 0,
			AttributeBitEnum_emergencyStop = 1,
            AttributeBitEnum_bumperStrip = 2,
            AttributeBitEnum_paylocad = 3,
            AttributeBitEnum_handleControl = 4,
            AttributeBitEnum_softEmergencyStop = 5,
            AttributeBitEnum_chargState = 6,
            AttributeBitEnum_safeLaserSwitch = 7
		};

        int m_agvID;
        float m_x, m_y, m_theta;
        float m_v, m_w;
        float m_energyLevel;
        float m_locationScore;
        AgvWorkMode m_workMode;
        slamSwitch m_slamSwitch;

        unsigned char     m_attribute;// 8位， 0 : 安全激光 ， 1 ： 急停拍下 2：安全触边  3：载货状态（0是没有货，1是有货），
                                        // 4：是否是手柄遥控状态, 5: 是否软急停， 6：是否在充电, 7:安全激光有没有打开
                                        
        AgvStatus() : m_agvID(0), m_x(0), m_y(0), m_theta(0), m_v(0), m_w(0), m_energyLevel(0), m_locationScore(0), m_attribute(0x00),
            m_workMode(AgvWorkMode::_ManualControlMode), m_slamSwitch(slamSwitch::_OpenSlamSwitchEnum_off)
        {
            int s = sizeof(AgvStatus);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvStatusHeadEnum;
        }

        AgvStatus(AgvStatus *ptr) : BaseData((BaseData *)ptr),
                                    m_agvID(ptr->m_agvID), m_x(ptr->m_x), m_y(ptr->m_y), m_theta(ptr->m_theta), m_v(ptr->m_v),
                                    m_w(ptr->m_w), m_energyLevel(ptr->m_energyLevel), m_locationScore(ptr->m_locationScore) , 
                                    m_attribute(ptr->m_attribute), m_workMode(ptr->m_workMode), m_slamSwitch(ptr->m_slamSwitch){}

        AgvStatus(AgvStatus &ptr) : BaseData(&ptr),
                                    m_agvID(ptr.m_agvID), m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta), m_v(ptr.m_v),
                                    m_w(ptr.m_w), m_energyLevel(ptr.m_energyLevel), m_locationScore(ptr.m_locationScore), 
                                    m_attribute(ptr.m_attribute), m_workMode(ptr.m_workMode), m_slamSwitch(ptr.m_slamSwitch) {}
        void setAttribute( const int val , AttributeBitEnum bit ) {    //  val =>  0/1     bit  ->  AttributeBitEnum
			if ( val == 0 ) 
				m_attribute = m_attribute & (~(   0x01 << bit )); 
			else if (val == 1)  
 				m_attribute = m_attribute | (0x01 << bit); 
		} 
		int getAttribute(AttributeBitEnum bit) {
			return (m_attribute >> bit) & 0x01;
		}
    };

    //任务相关的结构体的父类，不会单独出现使用
    struct JobBaseData
    {
        int m_agvID;
        int m_jobID;
        JobBaseData() : m_agvID(0), m_jobID(0) {}
        JobBaseData(const JobBaseData &obj) : m_agvID(obj.m_agvID), m_jobID(obj.m_jobID) {}
        JobBaseData(JobBaseData *ptr) : m_agvID(ptr->m_agvID), m_jobID(ptr->m_jobID) {}
    };

    //agv剩余时间
    struct AgvTaskRemainTime : public BaseData, public JobBaseData
    {
        int m_remainTime;
        unsigned char m_remainPercent;
        AgvTaskRemainTime() : m_remainTime(0), m_remainPercent(0)
        {
            int s = sizeof(AgvTaskRemainTime);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvRemainTimeHeadEnum;
        }
        AgvTaskRemainTime(AgvTaskRemainTime *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr),
                                                    m_remainTime(ptr->m_remainTime), m_remainPercent(ptr->m_remainPercent) {}
        AgvTaskRemainTime(AgvTaskRemainTime &ptr) : BaseData(&ptr), JobBaseData(&ptr),
                                                    m_remainTime(ptr.m_remainTime), m_remainPercent(ptr.m_remainPercent) {}
    };

    //agv任务执行状况
    struct AgvTaskStatus : public BaseData, public JobBaseData
    {
        enum TaskStatusEnum
        {
            LeaveStandby = 0x00,
            JobFinish = 0x01
        };
        unsigned char m_tskstatus;
        AgvTaskStatus() : m_tskstatus(LeaveStandby)
        {
            int s = sizeof(AgvTaskStatus);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvTaskStatusHeadEnum;
        }
        AgvTaskStatus(AgvTaskStatus *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr), m_tskstatus(ptr->m_tskstatus) {}
        AgvTaskStatus(AgvTaskStatus &ptr) : BaseData(&ptr), JobBaseData(&ptr), m_tskstatus(ptr.m_tskstatus) {}
    };

    //agv接收到任务时根据模式判断任务是否可以执行，并返回结果回复
    struct AgvTaskResult : public BaseData, public JobBaseData
    {
        enum TaskResultEnum
        {
            TaskResultSuccess = 0x00,
            TaskResultModeError,
            TaskResultMapFileNameError,
            TaskResultGoalNameError,
            TaskResultMacroNameError,
            TaskResultRepeatError,
            TaskResultCannotMoveError,
            TaskResultOhterError
        };
        // enum agvWorkModeEnum
        // {
        //     WorkMode = 0x00, //正常可工作模式
        //     SlamMode = 0x01, //手动建图模式
        //     EditMode = 0x02, //手动编辑模式
        //     MoveMode = 0x03  //手动可移动模式
        // };
        // 如果接收任务失败，则返回错误类型
        // enum ErrorType
        // {
        //     NoError = 0x00,         //没有错误
        //     WorkModeError = 0x01,   //工作模式不匹配
        //     AgvIdError = 0x02,      //AGV ID不匹配
        // };
        unsigned char m_result;     //任务执行结果
        unsigned char m_agvmode;    //agv的模式
        AgvTaskResult() : m_result(TaskResultSuccess), m_agvmode(static_cast<unsigned char>(AgvWorkMode::_ManualControlMode))
        {
            int s = sizeof(AgvTaskResult);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvTaskResultHeadEnum;
        }
        AgvTaskResult(AgvTaskResult *ptr) : 
            BaseData((BaseData *)ptr), 
            JobBaseData((JobBaseData *)ptr), 
            m_result(ptr->m_result), 
            m_agvmode(ptr->m_agvmode) {}
        AgvTaskResult(AgvTaskResult &ptr) : 
            BaseData(&ptr), 
            JobBaseData(&ptr), 
            m_result(ptr.m_result), 
            m_agvmode(ptr.m_agvmode) {}
    };

    //agv完成地图同步
    struct AgvSyncMap : public BaseData, public JobBaseData
    {
        AgvSyncMap()
        {
            int s = sizeof(AgvSyncMap);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvSyncMapHeadEnum;
        }
        AgvSyncMap(AgvSyncMap *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr) {}
        AgvSyncMap(AgvSyncMap &ptr) : BaseData(&ptr), JobBaseData(&ptr) {}
    };

    //agv区域申请，离开  目标点申请，离开,  等待点离开
    struct AgvArea : public BaseData, public JobBaseData
    {
        enum ActType
        {
            RequestActType = 0x00,
            LeaveActType = 0x01,
            RequestGoalActType = 0x02,
            LeaveGoalActType = 0x03
        };

        unsigned char m_actionType;
        unsigned char m_areaName[TCP_DATA_STR_LENGTH_AGV];
        AgvArea() : m_actionType(RequestActType)
        {
            int s = sizeof(AgvArea);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvAreaHeadEnum;
            memset(m_areaName, '\0', TCP_DATA_STR_LENGTH_AGV);
        }
        AgvArea(AgvArea *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr), m_actionType(ptr->m_actionType) {}
        AgvArea(AgvArea &ptr) : BaseData(&ptr), JobBaseData(&ptr), m_actionType(ptr.m_actionType) {}
    };

    //agv 上传激光数据
    struct AgvLaserinfo : public BaseData, public JobBaseData
    {
        enum ActiveType
        {
            ObjectType = 0x01,  //agv主动上传激光数据
            PositiveType = 0x00 //agv被动上传激光数据
        };

        float m_minAngle; //rad
        float m_maxAngle;
        float m_angleStep;
        float m_laserRange[TCP_LASER_LENGTH];
        int m_intensity[TCP_LASER_LENGTH];
        float m_x;
        float m_y;
        float m_theta;
        unsigned char m_active; //0x01为agv主动上传激光数据，0x00为被动上传激光数据，默认是被动

        AgvLaserinfo() : m_minAngle(0), m_maxAngle(0), m_angleStep(0), m_x(0), m_y(0), m_theta(0), m_active(PositiveType)
        {
            int s = sizeof(AgvLaserinfo);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvLaserInfoHeadEnum;
            memset(m_laserRange, 0, TCP_LASER_LENGTH * sizeof(float));
            memset(m_intensity, 0, TCP_LASER_LENGTH * sizeof(int));
        }
        AgvLaserinfo(AgvLaserinfo *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr),
                                          m_minAngle(ptr->m_minAngle), m_maxAngle(ptr->m_maxAngle), m_angleStep(ptr->m_angleStep),
                                          m_x(ptr->m_x), m_y(ptr->m_y), m_theta(ptr->m_theta), m_active(ptr->m_active)
        {
            memcpy(m_laserRange, ptr->m_laserRange, TCP_LASER_LENGTH * sizeof(float));
            memcpy(m_intensity, ptr->m_intensity, TCP_LASER_LENGTH * sizeof(float));
        }
        AgvLaserinfo(AgvLaserinfo &ptr) : BaseData(&ptr), JobBaseData(&ptr),
                                          m_minAngle(ptr.m_minAngle), m_maxAngle(ptr.m_maxAngle), m_angleStep(ptr.m_angleStep),
                                          m_x(ptr.m_x), m_y(ptr.m_y), m_theta(ptr.m_theta), m_active(ptr.m_active)
        {
            memcpy(m_laserRange, ptr.m_laserRange, TCP_LASER_LENGTH * sizeof(float));
            memcpy(m_intensity, ptr.m_intensity, TCP_LASER_LENGTH * sizeof(float));
        }
    };

    //agv回复上线/下线设置成功与否
    struct AgvSetOnOffResult : public BaseData, public JobBaseData
    {
        enum ResultType
        {
            FailResultType = 0x00,
            SuccessResultType = 0x01
        };

        enum LineType
        {
            OnlineType = 0x00,
            OfflineType = 0x01
        };

        unsigned char m_result;      //0x00 表示设置失败， 0x01表示设置成功
        unsigned char m_on_off_line; //0x00 表示下线， 0x01表示上线
        AgvSetOnOffResult() : m_result(FailResultType), m_on_off_line(OnlineType)
        {
            int s = sizeof(AgvSetOnOffResult);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvSetOnlineEnum;
        }
        AgvSetOnOffResult(AgvSetOnOffResult *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr),
                                                    m_result(ptr->m_result), m_on_off_line(ptr->m_on_off_line) {}
        AgvSetOnOffResult(AgvSetOnOffResult &ptr) : BaseData(&ptr), JobBaseData(&ptr),
                                                    m_result(ptr.m_result), m_on_off_line(ptr.m_on_off_line) {}
    };

    //agv拒绝Deploy连接
    struct AgvRejectConnect : public BaseData
    {
        unsigned char reject; //0x01   拒绝连接

        AgvRejectConnect()
        {
            int s = sizeof(AgvRejectConnect);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agv_reject_connect;
            reject = 0x01;
        }
        AgvRejectConnect(AgvRejectConnect *ptr) : BaseData((BaseData *)ptr)
        {
            reject = ptr->reject;
        }
        AgvRejectConnect(const AgvRejectConnect &ptr) : BaseData((BaseData)ptr)
        {
            reject = ptr.reject;
        }
    };

    //agv接收到任务的ID不匹配时反馈的结果
    struct GetWrongAgvIDResult : public BaseData , public JobBaseData
    {        
        GetWrongAgvIDResult()
        {
            int s = sizeof(GetWrongAgvIDResult);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvGetWrongIDTaskHeadEnum;
        }
        GetWrongAgvIDResult(GetWrongAgvIDResult *ptr) : 
            BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr) {}
        GetWrongAgvIDResult(GetWrongAgvIDResult &ptr) : 
            BaseData(&ptr), JobBaseData(&ptr) {}
    };

    //agv回复模式设置成功与否
    struct AgvSetModeResult : public BaseData , public JobBaseData
    {
        enum ResultType
        {
            FailResultType = 0x00,
            SuccessResultType = 0x01
        };
        unsigned char m_result; //0x00表示设置失败    0x01表示设置成功
        unsigned char m_mode;
        AgvSetModeResult() : m_result(FailResultType), m_mode(static_cast<unsigned char>(AgvWorkMode::_ManualControlMode))
        {
            int s = sizeof(AgvSetModeResult);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvSetModeResultHeadEnum;
        }
        AgvSetModeResult(AgvSetModeResult *ptr) : 
            BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr),
            m_result(ptr->m_result), 
            m_mode(ptr->m_mode) {}
        AgvSetModeResult(AgvSetModeResult &ptr) : 
            BaseData(&ptr), JobBaseData(&ptr),
            m_result(ptr.m_result), 
            m_mode(ptr.m_mode) {}
    };

    //agv回复模式设置成功与否
    struct AgvOpenSlamResult : public BaseData , public JobBaseData
    {
        enum OpenSlamSwitchEnum {
            OpenSlamSwitchEnum_on,
            OpenSlamSwitchEnum_off
        };
        enum ResultType
        {
            FailResultType = 0x00,
            SuccessResultType = 0x01
        };
        unsigned char m_result; //0x00表示设置失败    0x01表示设置成功
        unsigned char m_switch;
        AgvOpenSlamResult() : m_result(FailResultType), m_switch(static_cast<unsigned char>(SuccessResultType))
        {
            int s = sizeof(AgvOpenSlamResult);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _openSlamResultHeadEnum;
        }
        AgvOpenSlamResult(AgvOpenSlamResult *ptr) : 
            BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr),
            m_result(ptr->m_result), 
            m_switch(ptr->m_switch) {}
        AgvOpenSlamResult(AgvOpenSlamResult &ptr) : 
            BaseData(&ptr), JobBaseData(&ptr) ,
            m_result(ptr.m_result), 
            m_switch(ptr.m_switch) {}
    };

    //agv回复当前工作地图
    struct AgvCurrentMap : public BaseData, public JobBaseData
    {
        AgvCurrentMap()
        {
            int s = sizeof(AgvCurrentMap);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvCurrentMapEnum;
            memset(m_currMap, '\0', TCP_DATA_STR_LENGTH_AGV);
        }
        AgvCurrentMap(AgvCurrentMap *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr)
        {
            memcpy(m_currMap, ptr->m_currMap, TCP_DATA_STR_LENGTH_AGV);
        }
        AgvCurrentMap(AgvCurrentMap &ptr) : BaseData(&ptr), JobBaseData(&ptr)
        {
            memcpy(m_currMap, ptr.m_currMap, TCP_DATA_STR_LENGTH_AGV);
        }
        unsigned char m_currMap[TCP_DATA_STR_LENGTH_AGV];
    };

    struct DeployRequestMsg : public BaseData
	{
		enum RequestTypeEnum {
			StartRequestEnum = 0x00,
			StopRequestEnum  = 0x01
		};
		unsigned char m_requestType;
		DeployRequestMsg() : m_requestType(StartRequestEnum) {
			int s = sizeof(DeployRequestMsg);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _deployRequestMsgHeadEnum;//0xC0
			m_requestType = StartRequestEnum;
		}
		DeployRequestMsg(DeployRequestMsg* ptr ) : BaseData((BaseData*)ptr) , m_requestType(ptr->m_requestType){}
		DeployRequestMsg(DeployRequestMsg& ptr ) : BaseData(&ptr), m_requestType(ptr.m_requestType) {}
	};

    //向 DEPLOY 表明自己的身份， 是 AGV 还是 调度
	//_identifiedHeadEnum
	struct IdentifiedMsg : public BaseData
	{
		enum IdentifiedMsgEnum{
			IdentifiedMsgEnum_AGV,
			IdentifiedMsgEnum_CC
		};
		IdentifiedMsg(): m_whoami(IdentifiedMsgEnum_CC){
			int s = sizeof(IdentifiedMsg);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _identifiedHeadEnum; //0xD2
		}
		IdentifiedMsg(const IdentifiedMsg& other) : BaseData((BaseData*)&other ),m_whoami(other.m_whoami) {}
		IdentifiedMsg(const IdentifiedMsg* other) : BaseData((BaseData*)other ), m_whoami(other->m_whoami ) {}
		unsigned char m_whoami;
	};

    struct AgvSwitchMap : public BaseData, public JobBaseData
    {
        unsigned char m_newmap[TCP_DATA_STR_LENGTH_AGV];
        AgvSwitchMap()
        {
            int s = sizeof(AgvSwitchMap);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV * sizeof(unsigned char));
            m_head = _agvSwitchMapEnum;
            memset(m_newmap, '\0', TCP_DATA_STR_LENGTH_AGV);
        }
        AgvSwitchMap(AgvSwitchMap *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr) 
        {
            memcpy(m_newmap, ptr->m_newmap, TCP_DATA_STR_LENGTH_AGV);
        }
        AgvSwitchMap(AgvSwitchMap &ptr) : BaseData(&ptr), JobBaseData(&ptr) 
        {
            memcpy(m_newmap, ptr.m_newmap, TCP_DATA_STR_LENGTH_AGV);
        }
    };

    //上报异常
    struct AgvErrorReport : public BaseData, public JobBaseData
    {
        enum Error_grade
        {
            Error_NotInterrupt = 0, // 不中断
            Error_Interrupt         // 中断
        };
        enum Error_type
        {
            NoError = 0x00,
            Hardware_error = 0x01,  //硬件错误
            Software_error = 0x02,  //软件错误
            SubTask_error = 0x03,   //子任务执行错误
            UserDefine_error = 0x04 //用户自定义错误
        };

        enum Hardware_error_value
        {
            LaserSensor_error = 1, //激光错误  更多的是网路连接上的错误
            LaserSensor_fatal,     //激光错误 传感器不兼容
            SerialPort_error,      //串口错误
            Tcp_error,             //客户端错误
            Encoder_error,         //编码器错误
            LeftWheel_error,       // 左轮伺服故障
            RightWheel_error,      // 右轮伺服故障
            LeftWheelCom_error,    // 左轮伺服通信故障
            RightWheelCom_error,   // 右轮伺服通信故障
            TouchScreenCom_error,  // 触摸屏通信故障
            BMSCom_error,          // 电池BMS通信故障
            BMSComTimeout_error,   // 电池BMS通信超时
            MagneticCom_error,     // 磁条传感器通信故障
            ControlComTimeout_error,// 工控机通信超时
            ControlComData_error,  // 工控机通信数据错误
            ChargerCom_error       // 充电桩通信故障
        };  

        //agv error value
        enum Software_error_value
        {
            ObstacleStop = 1,    //停障
            Location_error,      //定位错误
            LoadMap_error,       //地图加载错误
            Configuration_error, //配置文件错误
            GrabMap_error,       //ftp地图抓取错误
            NoLaserData_error    //无激光数据
        };

        //subTask error value
        enum SubTask_error_value
        {
            SubTaskRunning = 1,     //子任务正在运行
            SubTaskSuccess,         //子任务成功
            ChargerNotFound,        //找不到充电桩
            MagneticNotFound,       //磁条找不到
            MagneticConnectFailed,  //磁条对接失败
            VshapeNotFound,         //未检测到模板V字形
            MapLoadFailed,          //地图加载失败
            TaskNameError,          //任务名称错误
            GlobalScanLocateFailed, //全局线段定位失败
            ReflectorLocateFailed,  //反光条定位失败
            VshapeLocateFailed,     //V字形定位失败
            StripsNotEnough,        //模型反光条个数少于2个
            BatteryCharge_error,    //电池充电异常
            ConnectIpPortFailed,    //网路连接失败
            ReceiveIpTextFailed     //从网路接收数据失败
        };

        //UserDefine_error
        enum UserDefine_error_value
        {
            ErrorJump = 1, //异常跳转
            CheckIoTimeOut //CheckIo中超时异常
        };

        unsigned char m_errorType;                        //异常的大类
        unsigned char m_suberrorType;                     //异常的小类(只有子任务异常才会有小类)
        unsigned char m_ifinterrupt;                      //是否中断，0为不中断异常，1为中断异常
        unsigned char m_errorNum;                         //异常编号，这边上报的异常编号
        unsigned char m_goal[TCP_DATA_STR_LENGTH_AGV];    //发生异常的工位点名字，不一定会有；
        unsigned char m_subtask[TCP_DATA_STR_LENGTH_AGV]; // 子任务名字

        AgvErrorReport() : m_errorType(NoError), m_suberrorType(ObstacleStop), m_ifinterrupt(Error_NotInterrupt), m_errorNum(0)
        {
            int s = sizeof(AgvErrorReport);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvErrReptHeadEnum;
            memset(m_goal, '\0', TCP_DATA_STR_LENGTH_AGV);
            memset(m_subtask, '\0', TCP_DATA_STR_LENGTH_AGV);
        }
        AgvErrorReport(AgvErrorReport *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr),
                                              m_errorType(ptr->m_errorType), m_suberrorType(ptr->m_suberrorType),
                                              m_ifinterrupt(ptr->m_ifinterrupt), m_errorNum(ptr->m_errorNum)
        {
            memcpy(m_goal, ptr->m_goal, TCP_DATA_STR_LENGTH_AGV);
            memcpy(m_subtask, ptr->m_subtask, TCP_DATA_STR_LENGTH_AGV);
        }
        AgvErrorReport(AgvErrorReport &ptr) : BaseData(&ptr), JobBaseData(&ptr),
                                              m_errorType(ptr.m_errorType), m_suberrorType(ptr.m_suberrorType),
                                              m_ifinterrupt(ptr.m_ifinterrupt), m_errorNum(ptr.m_errorNum)
        {
            memcpy(m_goal, ptr.m_goal, TCP_DATA_STR_LENGTH_AGV);
            memcpy(m_subtask, ptr.m_subtask, TCP_DATA_STR_LENGTH_AGV);
        }
    };

    //异常结构体
    struct Error_information
    {
        int m_agvID;
        int m_jobID;
        unsigned int m_errorNum;                          //异常编号      从2开始
        unsigned int m_errorType;                         //异常大类
        unsigned int m_suberrorType;                      //异常小类
        unsigned int m_ifinterrupt;                       //0表示不中断， 1表示中断
        unsigned char m_goal[TCP_DATA_STR_LENGTH_AGV];    //发生异常的工位点名
        unsigned char m_subtask[TCP_DATA_STR_LENGTH_AGV]; //发送异常的子任务名

        Error_information() : m_agvID(0), m_jobID(0)
        {
            m_errorNum = 0;
            m_errorType = AgvErrorReport::NoError;
            m_suberrorType = AgvErrorReport::SubTaskRunning;
            m_ifinterrupt = AgvErrorReport::Error_NotInterrupt;
            memset(m_goal, '\0', TCP_DATA_STR_LENGTH_AGV * sizeof(unsigned char));
            memset(m_subtask, '\0', TCP_DATA_STR_LENGTH_AGV * sizeof(unsigned char));
        }
        Error_information(Error_information *ptr) : m_agvID(ptr->m_agvID), m_jobID(ptr->m_jobID),
                                                    m_errorNum(ptr->m_errorNum), m_errorType(ptr->m_errorType),
                                                    m_suberrorType(ptr->m_suberrorType), m_ifinterrupt(ptr->m_ifinterrupt)
        {
            memcpy(m_goal, ptr->m_goal, TCP_DATA_STR_LENGTH_AGV);
            memcpy(m_subtask, ptr->m_subtask, TCP_DATA_STR_LENGTH_AGV);
        }
        Error_information(const Error_information &ptr) : m_agvID(ptr.m_agvID), m_jobID(ptr.m_jobID),
                                                          m_errorNum(ptr.m_errorNum), m_errorType(ptr.m_errorType),
                                                          m_suberrorType(ptr.m_suberrorType), m_ifinterrupt(ptr.m_ifinterrupt)
        {
            memcpy(m_goal, ptr.m_goal, TCP_DATA_STR_LENGTH_AGV);
            memcpy(m_subtask, ptr.m_subtask, TCP_DATA_STR_LENGTH_AGV);
        }

        inline bool operator==(const Error_information &rhs)
        {
            return (m_errorType == rhs.m_errorType && m_suberrorType == rhs.m_suberrorType && m_ifinterrupt == rhs.m_ifinterrupt);
        }
    };

    //清除异常,及取消任务 继续任务的消息回复
    struct AgvErrorDelete : public BaseData, public JobBaseData
    {
        enum DelResultTypeEnum
        {
            DelFailedEnum = 0x00,
            DelSuccessEnum = 0x01
        };

        unsigned char m_errorNum;  // 0表示继续任务， 1表示取消任务, 2开始表示删除的异常的编号
        unsigned char m_delResult; //删除的结果，0表示删除失败，1表示删除成功；

        AgvErrorDelete() : m_errorNum(0), m_delResult(DelFailedEnum)
        {
            int s = sizeof(AgvErrorDelete);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _avgErrDelHeadEnum;
        }
        AgvErrorDelete(AgvErrorDelete *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr),
                                              m_errorNum(ptr->m_errorNum), m_delResult(ptr->m_delResult) {}
        AgvErrorDelete(AgvErrorDelete &ptr) : BaseData(&ptr), JobBaseData(&ptr),
                                              m_errorNum(ptr.m_errorNum), m_delResult(ptr.m_delResult) {}
    };

    //agv询问配置文件路径，任务无关
    struct AgvAskConfig : public BaseData
    {
        int m_agvID;
        AgvAskConfig() : m_agvID(0)
        {
            int s = sizeof(AgvAskConfig);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvAskConfigHeadEnum;
        }
        AgvAskConfig(AgvAskConfig *ptr) : BaseData((BaseData *)ptr) {}
        AgvAskConfig(AgvAskConfig &ptr) : BaseData(&ptr) {}
    };

    struct AgvAttribute : public BaseData
	{
		enum AttributeEnum
		{
			AgvAttributeEnum_x0 = 0,
            AgvAttributeEnum_y0 = 1,
			AgvAttributeEnum_x1 = 2,
            AgvAttributeEnum_y1 = 3,
			AgvAttributeEnum_x2 = 4,
			AgvAttributeEnum_y2 = 5,
			AgvAttributeEnum_x3 = 6,
			AgvAttributeEnum_y3 = 7,
			AgvAttributeEnum_autocharge = 8,
			AgvAttributeEnum_stopcharge = 9,
            AgvAttributeEnum_avoidobs   = 10
		};
		int    m_agvID;
		int     m_para;
		int     m_attribute_type;
		AgvAttribute() {
			int s = sizeof(AgvAttribute);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _agvAttributeHeadEnum;
			m_para = 0;
			m_attribute_type = AgvAttributeEnum_x0;
			m_agvID = 0;
		}
		AgvAttribute( AgvAttribute* ptr ) : BaseData( (BaseData*)ptr ), m_para( ptr->m_para ) , m_attribute_type( ptr->m_attribute_type), m_agvID(ptr->m_agvID){}
		AgvAttribute( AgvAttribute& ptr ) : BaseData( (BaseData&)ptr ), m_para( ptr.m_para  ) , m_attribute_type( ptr.m_attribute_type ), m_agvID(ptr.m_agvID) {}
		
	};

    //任务恢复开始/结束 资源对齐
    struct AgvRecover : public BaseData, public JobBaseData
    {
        enum RecoverType
        {
            RecoverType_begin = 0x00,
            RecoverType_end = 0x01
        };
        unsigned char m_recover_type;
        AgvRecover() : m_recover_type(RecoverType_begin)
        {
            int s = sizeof(AgvRecover);
            memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
            m_head = _agvRecoverHeadEnum;
        }
        AgvRecover(AgvRecover *ptr) : BaseData((BaseData *)ptr), JobBaseData((JobBaseData *)ptr), m_recover_type(ptr->m_recover_type) {}
        AgvRecover(AgvRecover &ptr) : BaseData(&ptr), JobBaseData(&ptr), m_recover_type(ptr.m_recover_type) {}
    };

    //任务恢复之后对齐资源
    struct AgvRecoverSource : public BaseData, public JobBaseData
	{
		enum SourceType {
			SourceType_goal = 0x00,
			SourceType_area = 0x01,
			SourceType_currentmap = 0x02,
            SourceType_job = 0x03,
            SourceType_chargerStation = 0x04
		};
        enum OccupTypeEnum {
            OccupTypeEnum_accept = 0x00,
            OccupTypeEnum_refuse = 0x01,
            OccupTypeEnum_noresponse = 0x02
        };
		unsigned char m_sourceType;
		unsigned char m_sourceName[TCP_DATA_STR_LENGTH_AGV];
        unsigned char m_occupType;
		AgvRecoverSource() : m_sourceType(SourceType_goal), m_occupType(OccupTypeEnum_accept) {
			int s = sizeof(AgvRecoverSource);
			memcpy(m_dataLength, &s , TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _agvRecoverSourceHeadEnum;
			memset(m_sourceName, '\0', TCP_DATA_STR_LENGTH_AGV);
		}
		AgvRecoverSource(AgvRecoverSource* ptr) : 
        BaseData((BaseData*)ptr), JobBaseData((JobBaseData*)ptr), 
        m_sourceType(ptr->m_sourceType),  m_occupType(ptr->m_occupType) {
			memcpy(m_sourceName, ptr->m_sourceName, TCP_DATA_STR_LENGTH_AGV);
		}
		AgvRecoverSource(AgvRecoverSource& ptr) : 
        BaseData(&ptr), JobBaseData(&ptr), 
        m_sourceType(ptr.m_sourceType), m_occupType(ptr.m_occupType){
			memcpy(m_sourceName, ptr.m_sourceName, TCP_DATA_STR_LENGTH_AGV);
		}

		//_agvRecoverSourceHeadEnum
	};

    //agv询问cc时间
	struct AgvAskTime :public BaseData 
	{
		int              m_agvID;
		AgvAskTime(): m_agvID(0) {
			int s = sizeof(AgvAskTime);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _agvAskTimeHeadEnum;
		}
		AgvAskTime(const AgvAskTime* ptr) : BaseData((BaseData*)ptr), m_agvID(ptr->m_agvID) {}
		AgvAskTime(const AgvAskTime& ptr) : BaseData((BaseData*)&ptr),m_agvID(ptr.m_agvID) {}
	};

    struct AgvAskMapVersion : public BaseData
	{
		int              m_agvID;
		unsigned char     m_mapfile[TCP_DATA_STR_LENGTH_AGV];
		AgvAskMapVersion() {
			m_head = _agvAskMapVersionHeadEnum;
			int s = sizeof(AgvAskMapVersion);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			memset(m_mapfile,    '\0', TCP_DATA_STR_LENGTH_AGV);
			m_agvID = 0;
		}
		AgvAskMapVersion(const AgvAskMapVersion* ptr) : BaseData((BaseData*)ptr) {
			memcpy(m_mapfile, ptr->m_mapfile, TCP_DATA_STR_LENGTH_AGV);
			m_agvID = ptr->m_agvID;
		}
		AgvAskMapVersion(const AgvAskMapVersion& ptr) : BaseData((BaseData&)ptr) {
			memcpy(m_mapfile, ptr.m_mapfile, TCP_DATA_STR_LENGTH_AGV);
			m_agvID = ptr.m_agvID;
		}
	};

    struct AgvStCannotReach : public BaseData, public JobBaseData
	{
		AgvStCannotReach() : m_agvID(0) {
			m_head = _agvStandbyCannotReachHeadEnum;//0xB4
			int s = sizeof(AgvStCannotReach);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			memset(m_source, '\0', TCP_DATA_STR_LENGTH_AGV);
			memset(m_standby, '\0', TCP_DATA_STR_LENGTH_AGV);
			m_source_type = AreaSourceType;
		}
		AgvStCannotReach(const AgvStCannotReach* ptr) : BaseData((BaseData*)ptr), JobBaseData((JobBaseData*)ptr) {
			memcpy(m_source, ptr->m_source, TCP_DATA_LENGTH_SIZE_AGV);
			memcpy(m_standby, ptr->m_standby, TCP_DATA_LENGTH_SIZE_AGV);
			m_agvID = ptr->m_agvID;
			m_source_type = ptr->m_source_type;
		}
		AgvStCannotReach(const AgvStCannotReach& ptr) : BaseData((BaseData&)ptr), JobBaseData((JobBaseData&)ptr) {
			memcpy(m_source, ptr.m_source, TCP_DATA_LENGTH_SIZE_AGV);
			memcpy(m_standby, ptr.m_standby, TCP_DATA_LENGTH_SIZE_AGV);
			m_agvID = ptr.m_agvID;
			m_source_type = ptr.m_source_type;
		}

		enum SourceType {
			AreaSourceType = 0x00,
			GoalSourceType = 0x01
		};
		int              m_agvID;
		unsigned char     m_source[TCP_DATA_STR_LENGTH_AGV];
		unsigned char     m_standby[TCP_DATA_STR_LENGTH_AGV];
		unsigned char     m_source_type;
	};
    //小车重连的时候就发送这条消息，告诉调度这次是重连还是重启，与任务无关
	struct AgvRestart : public BaseData
	{
		enum RestartTypeEnum {
			RestartTypeEnum_Restart   = 0x00,
			RestartTypeEnum_Reconnect = 0x01
		};
		int                       m_agvID;
		unsigned char              m_RestartType;
		AgvRestart() :m_agvID(0), m_RestartType(RestartTypeEnum_Restart) {
			int s = sizeof(AgvRestart);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _agvRestartHeadEnum;
		}
		AgvRestart(const AgvRestart* ptr):BaseData((BaseData*)ptr),m_agvID(ptr->m_agvID) , m_RestartType(ptr->m_RestartType){}
		AgvRestart(const AgvRestart& ptr):BaseData((BaseData*)&ptr),m_agvID(ptr.m_agvID) , m_RestartType(ptr.m_RestartType){}
	};
    //小车到点的消息，到达工位点就发送这个消息，不需要等到完成任务才发送
	struct AgvArrive : public BaseData
	{
		int              m_agvID;
		unsigned char     m_goalname[TCP_DATA_STR_LENGTH_AGV];
		AgvArrive() {
			int s = sizeof(AgvArrive);
			memcpy(m_dataLength, &s, TCP_DATA_STR_LENGTH_AGV);
			m_head = _agvArriveHeadEnum;//0xB6
			memset(m_goalname, '\0', TCP_DATA_STR_LENGTH_AGV);
			m_agvID = 0;
		}
		AgvArrive(const AgvArrive* ptr) : BaseData((BaseData*)ptr) , m_agvID(ptr->m_agvID) {
			memcpy(m_goalname, ptr->m_goalname, TCP_DATA_STR_LENGTH_AGV);
		}
		AgvArrive(const AgvArrive& ptr) : BaseData((BaseData&)ptr), m_agvID(ptr.m_agvID) {
			memcpy(m_goalname, ptr.m_goalname, TCP_DATA_STR_LENGTH_AGV);
		}
	};
    struct AgvLastJobStatus : public BaseData, public  JobBaseData
	{
		enum LastJobStatusEnum {
			LastJobStatusFinish  = 0x00 ,
			LastJobStatusRunning = 0x01 
		};
		unsigned char    m_lastjobstatus;
		AgvLastJobStatus() {
			int s = sizeof(AgvLastJobStatus);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _agvLastJobStatusHeadEnum; //0xB7
			m_lastjobstatus = LastJobStatusFinish;
		}
		AgvLastJobStatus(AgvLastJobStatus *ptr):BaseData((BaseData*)ptr), JobBaseData((JobBaseData*)ptr),m_lastjobstatus(ptr->m_lastjobstatus){}
		AgvLastJobStatus(AgvLastJobStatus &ptr) : BaseData(&ptr), JobBaseData(&ptr), m_lastjobstatus(ptr.m_lastjobstatus) {}
	};
	struct AgvFinishReadLocalConfig : public BaseData
	{
		int               m_agvID;
		AgvFinishReadLocalConfig() :m_agvID(0) {
			int s = sizeof(AgvFinishReadLocalConfig);
			memcpy(m_dataLength, &s, TCP_DATA_LENGTH_SIZE_AGV);
			m_head = _agvFinishReadLocalConfigHeadEnum; //0xBB
		}
		AgvFinishReadLocalConfig(  AgvFinishReadLocalConfig* ptr) : BaseData((BaseData*)ptr) , m_agvID(ptr->m_agvID) { }
		AgvFinishReadLocalConfig(  AgvFinishReadLocalConfig& ptr) : BaseData((BaseData&)ptr), m_agvID(ptr.m_agvID) { }
	};
} // end namespace MSG_AGV

#ifdef __cplusplus
}
#endif

#endif // MSGAGV
