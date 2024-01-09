#include <math.h>
#include <algorithm>


#include "otg_filter.h"

namespace motionplanner{

#define SIGN1(a) copysign(1.0, a)
using std::max;
using std::min;

OtgFilter::OtgFilter()
{

}

OtgFilter::~OtgFilter()
{
	
}

/// \brief otgRunBase 计算周期的PVA
int32_t OtgFilter::otgRunBase(const LimitParam &limit, double ts, VelParam &nk)
{
	VelParam ek, eMin, eMax;
    //OtgLimitParam *limit = &(alimit);
    double s1, s2, s3;
    double s1Sign;
    double vPos, vNeg;
	double uc, uvMin, uvMax, uk;
	double U = limit.jMax;
    ek.d = (qk.d - rk.d) / U;
    ek.v = (qk.v - rk.v) / U;
    ek.a = (qk.a - rk.a) / U;
    eMin.v = (-limit.vMax - rk.v) / U;
    eMin.a = (-limit.aMax - rk.a) / U;
    eMax.v = (limit.vMax - rk.v) / U;
    eMax.a = (limit.aMax - rk.a) / U;
    s1 = ek.v + ek.a*fabs(ek.a)*0.5;
    s1Sign = SIGN1(s1);
	s2 = ek.d + ek.v*ek.a*s1Sign - pow(ek.a, 3.0) / 6.0 * (1.0 - 3.0 * fabs(s1Sign))
		+ s1Sign / 4.0 * sqrt(2.0 * pow((ek.a*ek.a + 2.0 * ek.v*s1Sign), 3.0));
	vPos = ek.d - eMax.a*(ek.a*ek.a - 2.0 * ek.v) / 4.0
		- pow((ek.a * ek.a - 2.0 * ek.v), 2.0) / (8.0 * eMax.a) - ek.a*(3.0 * ek.v - ek.a*ek.a) / 3.0;
	vNeg = ek.d - eMin.a*(ek.a*ek.a + 2.0 * ek.v) / 4.0
		- pow((ek.a * ek.a + 2.0 * ek.v), 2.0) / (8.0 * eMin.a) + ek.a*(3.0 * ek.v + ek.a*ek.a) / 3.0;

    if(ek.a <= eMax.a && ek.v<=ek.a*ek.a*0.5-eMax.a*eMax.a) s3 = vPos;
    else if(ek.a >= eMin.a && ek.v>=eMin.a*eMin.a-ek.a*ek.a*0.5) s3 = vNeg;
    else s3 = s2;

    uc = -U * SIGN1( s3 + (1.0-fabs(SIGN1(s3))) * (s1 + (1.0-fabs(s1Sign))*ek.a));
	uvMin = max(-U * SIGN1(ek.a - eMin.a), min(-U * SIGN1(ek.a*fabs(ek.a) + 2.0 * (ek.v - eMin.v)
		+ ek.a*(1.0 - fabs(SIGN1(ek.a*fabs(ek.a) + 2.0 * (ek.v - eMin.v))))), -U * SIGN1(ek.a - eMax.a)));
	uvMax = max(-U * SIGN1(ek.a - eMin.a), min(-U * SIGN1(ek.a*fabs(ek.a) + 2.0 * (ek.v - eMax.v)
		+ ek.a*(1.0 - fabs(SIGN1(ek.a*fabs(ek.a) + 2.0 * (ek.v - eMax.v))))), -U * SIGN1(ek.a - eMax.a)));
	uk = max(uvMin, min(uc, uvMax));

	// 积分计算
	/*qk.a += cycle_time * uk;
	qk.v += cycle_time * qk.a;
	qk.p += cycle_time * qk.v;*/
	nk.a = qk.a + ts * uk;
	nk.v = qk.v + ts * nk.a;
	nk.d = qk.d + ts * nk.v;
    return OTG_SUCCESS;
}



/// \brief reset 复位 清理buff
int32_t OtgFilter::reset()
{
	qk = VelParam{ 0, 0, 0 ,0};
	rk = qk;
	return 0;
}


/// \brief 获取目标点
/// \param[in] dt 时间差单位ms
/// \param[in] param 参数包括当前状态、目标状态
/// \param[in] next 下一个周期的位置速度
int32_t OtgFilter::runCycleS1(uint64_t dt, const LimitParam& limit, const VelParam& target)
{
	rk = target;
	uint32_t calcTimes = (uint32_t)floor((dt) / (1000.0 * cycle_time));  // cycle_time 是积分时间 0.1ms，50ms需要500次积分
	for (uint32_t i = 0; i < calcTimes; i++) {
		otgRunBase(limit, cycle_time, qk);
		if (fabs(qk.d - rk.d) < pos_epsilon
			&& fabs(qk.v - rk.v) < vel_epsilon) {
			qk.a = 0;
			qk.v = 0.01;
			return OTG_DONE_ALL;
		}
	}
	if(qk.v < 0.01) qk.v = 0.01;
	return OTG_SUCCESS;
}

}

