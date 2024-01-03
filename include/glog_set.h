#ifndef GLOG_SET_H
#define GLOG_SET_H

#include <glog/logging.h>
#include <glog/raw_logging.h>

#ifdef __cplusplus
extern "C" {
#endif


class GLog_set
{
public:
    //GLOG配置：
    GLog_set(char* program);
    //GLOG内存清理：
    ~GLog_set();
};

#ifdef __cplusplus
}
#endif
#endif 

