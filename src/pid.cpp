#include "pid.hpp"

pid::pid()
{
    reset();   
}

void pid::setGains(double p,double i,double d)
{
    this -> p = p;
    this -> i = i;
    this -> d = d;
}

void pid::setDt(double dt)
{
    this -> dt = dt;
}

void pid::setTarget(double target)
{
    this -> target = target;
    error_ = 0; //清空上次误差
    integral = 0;   // 清空积分
    is_first = true;    //首次计算
}

double pid::compute(double current)
{

    double error = target - current;
    double abs_error =std::fabs(error);
    if(abs_error < dead_zone_threshold && dead_zone_flag) error = 0;  //死区

    double P = p * error;
    
    //积分分离
    double I = i * integral;
    if(!(abs_error < integral_sepration_threshold && integral_sepration_flag))
    {
        integral += error * dt;
        if(integral >= integral_lim) integral = integral_lim;
        if(integral <= -integral_lim) integral = -integral_lim;
    }
    else I = 0;

    //微分先行和微分滤波
    double D;
    if(d_first_flag)
    {
        if(is_first)
        {
            D = 0;
            is_first = false;
        }
        else 
        {
            D = d * (current_ - current) / dt;
            if(d_filter_flag)   D = D * d_filter_value + D_ * (1 - d_filter_value);
        }
    }
    else
    {
        D = d * (error - error_) / dt;
        if(d_filter_flag)   D = D * d_filter_value +  D_ * (1 - d_filter_value);
    }
    
    double output = P + I + D;

    //输出限幅
    if(max_output && std::fabs(output) > max_output)
    {
            double o_ = output;
            output = max_output;
            if(o_ < 0) output = -max_output;
    }
    if(min_output && std::fabs(output) < min_output)
    {
            double o_ = output;
            output = min_output;
            if(o_ < 0) output = -min_output;
    }

    error_ = error;
    current_ = current;
    D_ = D;

    return output;
    
}

double pid::compute_(double error)
{

    double abs_error =std::fabs(error);
    if(abs_error < dead_zone_threshold && dead_zone_flag) error = 0;  //死区

    double P = p * error;
    
    //积分分离
    double I = i * integral;
    if(!(abs_error > integral_sepration_threshold && integral_sepration_flag))
    {
        integral += error * dt;
        if(integral >= integral_lim) integral = integral_lim;
        if(integral <= -integral_lim) integral = -integral_lim;
    }
    else I = 0;
    
    //微分滤波
    double D;
    D = d * (error - error_) / dt;
    if(d_filter_flag)   D = D * d_filter_value +  D_ * (1 - d_filter_value);
    
    double output = P + I + D;

    //输出限幅
    if(max_output && std::fabs(output) > max_output)
    {
            double o_ = output;
            output = max_output;
            if(o_ < 0) output = -max_output;
    }
    if(min_output && std::fabs(output) < min_output)
    {
            double o_ = output;
            output = min_output;
            if(o_ < 0) output = -min_output;
    }

    error_ = error;
    D_ = D;

    return output;
    
}

void pid::setIntegralLimit(double lim)
{
    this -> integral_lim = lim;
}

void pid::setOutputLimit(double min,double max)
{
    this -> min_output = min;
    this -> max_output = max;
}

void pid::reset()
{
    error_ = 0;
    current_ = 0;
    D_ = 0;
    p = 0;
    i = 0;
    d = 0;
    target = 0;
    integral = 0;
    integral_lim = 0;
    max_output = 0;
    dt = 1;
    integral_sepration_flag = false , integral_sepration_threshold = 0;
    dead_zone_flag = false , dead_zone_threshold = 0;
    d_filter_flag = false , d_filter_value = 0;
    d_first_flag = false , is_first = true;
}

void pid::setAdvancedFlag(bool integral_sepration_flag, bool dead_zone_flag, bool d_filter_flag, bool d_first_flag)
{
    this -> integral_sepration_flag = integral_sepration_flag;
    this -> dead_zone_flag = dead_zone_flag;
    this -> d_filter_flag = d_filter_flag;
    this -> d_first_flag = d_first_flag;
};

void pid::setAdvancedThreshold(double integral_sepration_threshold, double dead_zone_threshold, double d_filter_value)
{
    this -> integral_sepration_threshold = integral_sepration_threshold;
    this -> dead_zone_threshold = dead_zone_threshold;
    this -> d_filter_value = d_filter_value;
}

double pid::getTarget()
{
    return target;
}