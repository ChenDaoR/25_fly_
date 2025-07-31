#ifndef __PID_HPP__
#define __PID_HPP__

#include <cmath>

class pid
{
private:
    double p,i,d;   //pid参数
    double integral , integral_lim; //积分项，积分限幅
    double error_;  //误差记录
    double target;   //目标值记录
    double dt;  //设置时间步长
    double max_output , min_output;    //输出限制
    double integral_sepration_flag , integral_sepration_threshold; //积分分离
    double dead_zone_flag , dead_zone_threshold;    //死区
    double d_filter_flag , d_filter_value , D_;  //微分低通滤波
    double d_first_flag , current_ , is_first;  //微分先行


public:

    pid();  //构造函数
    void setGains(double ,double ,double);  //设置参数
    void setDt(double);    //设置时间步长
    void setTarget(double );  //设置计算目标

    double compute(double );    //位置式计算输出
    double compute_(double  );  //误差式计算输出
    
    void setIntegralLimit(double );  //积分限幅 0代表不限幅
    void setOutputLimit(double , double);    //输出限幅

    void reset();   //重置

    void setAdvancedFlag(bool ,bool ,bool ,bool);   //高级功能启动
    void setAdvancedThreshold(double ,double ,double ); //高级功能阈值设定

    double getTarget();
    

};


#endif