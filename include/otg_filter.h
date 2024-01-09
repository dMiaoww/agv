#ifndef OTG_FILTER_H
#define OTG_FILTER_H
#include <stdint.h>

namespace motionplanner{
class OtgFilter
{
public:
	OtgFilter();
	~OtgFilter();

	enum OtgReturn {
		OTG_FAILD = -1,		///< OTG计算失败
		OTG_SUCCESS = 0,	///< OTG计算成功
		OTG_DONE_ALL = 1,	///< OTG所有路径完成
		OTG_DONE_THIS = 2,	///< OTG当前运动完成
	};

	/// \brief OTG限制信息
	/// \note vMax aMax jMax均必须 > 1e-5
	struct LimitParam {
		LimitParam() {}
		LimitParam(const double& v, const double& a, const double& j) : vMax(v), aMax(a), jMax(j) {
		}
		double vMax; 	///< 最大速度
		double aMax;	///< 加速的最大加速度
		double jMax;	///< 最大jerk
	};

	/// \brief 速度参数
	struct VelParam
	{
		double d;		///< 位置
		double v;		///< 速度
		double a;		///< 加速度
		double j;		///< 加加速度
	};


	struct RtLimit
	{
		RtLimit() {}
		RtLimit(const double& _v, const double& _s = 0, const double& _a = 0) : vMax(_v), disToZero(_s), aMax(_a) { aMin = 0; }
		double vMax;		///< 实时的最大速度限制
		double disToZero;	///< 减速到0的最长距离 ditToZero <= 0 表示不限制
		double aMax;		///< 实时最大加速度限制, aMax <= 0表示不限制
		double aMin;		///< 最小加速度
	};

public:

	/// \brief reset 复位 清理buff
	int32_t reset();


	int32_t runCycleS1(uint64_t dt, const LimitParam& limit, const VelParam& target);

private:
	/// \brief otgRunBase 计算周期的PVA
	int32_t otgRunBase(const LimitParam &limit, double ts, VelParam &nk);

public:
	double cycle_time = 1e-4;	///< 计算周期 单位秒 取值1e-4
	double pos_epsilon = 1e-5;	///< 位置精度 取1e-5
	double vel_epsilon = 1e-4;	///< 停止的速度精度 1e-4
	LimitParam alimit;	///< 当前的处理完成最终限制
	VelParam rk;		///< 目标位置
	VelParam qk;		///< 当前的状态
};

}

#endif // OTG_FILTER_H_