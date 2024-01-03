#include "glog_set.h"
#include <stdlib.h>
//配置输出日志的目录：
#define LOGDIR "log"
#define MKDIR "mkdir -p log"

//将信息输出到单独的文件和 LOG(ERROR)
void SignalHandle(const char* data, int size)
{
    std::string str = std::string(data,size);
    LOG(ERROR) << str;
}

GLog_set::GLog_set(char* program)
{
  std::string info_n(LOGDIR"/INFO_"), warning_n(LOGDIR"/WARNING_"), 
    error_n(LOGDIR"/ERROR_"), fatal_n(LOGDIR"/FATAL_");
  if(system(MKDIR) == -1) {
    info_n = std::string("./INFO_");
    warning_n = std::string("./WARNING_");
    error_n = std::string("./ERROR_");
    fatal_n = std::string("./FATAL_");
  }

  google::InitGoogleLogging(program);

	// #ifndef NDEBUG
    google::SetStderrLogging(google::INFO);                         //设置级别高于 google::INFO 的日志同时输出到屏幕
    google::SetLogDestination(google::INFO, info_n.c_str());         //设置 google::INFO 级别的日志存储路径和文件名前缀
	// #else
  //   google::SetStderrLogging(google::WARNING);                         //设置级别高于 google::INFO 的日志同时输出到屏幕
  // #endif

  FLAGS_colorlogtostderr=true;                                    //设置输出到屏幕的日志显示相应颜色  

  google::SetLogDestination(google::WARNING, warning_n.c_str());   //设置 google::WARNING 级别的日志存储路径和文件名前缀
  google::SetLogDestination(google::ERROR, error_n.c_str());       //设置 google::ERROR 级别的日志存储路径和文件名前缀
  google::SetLogDestination(google::FATAL, fatal_n.c_str());       //设置 google::FATAL 级别的日志存储路径和文件名前缀
  FLAGS_logbufsecs =0;                            //缓冲日志输出，默认为30秒，此处改为立即输出
  FLAGS_max_log_size =100;                        //最大日志大小为 100MB
  FLAGS_stop_logging_if_full_disk = true;         //当磁盘被写满时，停止日志输出
  //google::SetLogFilenameExtension("91_");         //设置文件名扩展，如平台？或其它需要区分的信息
  google::InstallFailureSignalHandler();          //捕捉 core dumped
  google::InstallFailureWriter(&SignalHandle);    //默认捕捉 SIGSEGV 信号信息输出会输出到 stderr，可以通过下面的方法自定义输出>方式：
}

GLog_set::~GLog_set()
{
  google::ShutdownGoogleLogging();
} 

