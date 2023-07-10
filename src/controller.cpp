#include<controller.h>
#include<Arduino.h>

controllers::PID::PID(float &kp, float &kd, float &ki, int &controller_freq, String &type, bool &hasLimt)
    : kp_{kp},kd_{kd}, ki_{ki},controller_freq_{controller_freq},PIDVariant_{type}, hasLimt_{hasLimt}
    {
        start_time = millis();
    };

controllers::PID::PID(){};

void controllers::PID::Setkp(float &kp){

    kp_ = kp;
    
};

void controllers::PID::Setkd(float &kd){

    kd_ = kd;

};

void controllers::PID::Setki(float &ki){

    ki_ = ki;

};

void controllers::PID::SetCtrlr_freq(int &freq){

    controller_freq_ = freq;

};

void controllers::PID::SetPIDVariant(String &type){

    PIDVariant_ = type;

};

void controllers::PID::setLimits(double &Upperlim, double &Lowerlim){

    UpperLimt_ = Upperlim;
    LowerLimt_ = Lowerlim;

}

void controllers::PID::computeOuput(float &input, float &setPoint){
    
    end_time = millis();
    float ctrld_output = 0;

    if((end_time - start_time) > dt){

        error = input - setPoint;

        if(PIDVariant_ == "P"){

            ctrld_output = kp_ * error + input;

            if(hasLimt_){
                if(ctrld_output >= UpperLimt_) output = UpperLimt_; 
                if(ctrld_output <= LowerLimt_) output = LowerLimt_;
                else output = ctrld_output;
                prev_error = error;
            }

            else{

                output = ctrld_output;
                prev_error = error;
            
            }
        }

        if(PIDVariant_ == "PD"){

            ctrld_output = kp_ * error + kd_ * (error - prev_error)/dt + input;

            if(hasLimt_){
                if(ctrld_output >= UpperLimt_) output = UpperLimt_;
                if(ctrld_output <= LowerLimt_) output = LowerLimt_;
                else output = ctrld_output;

            }

            else{
                
                output = ctrld_output;
                prev_error = error;
                
                }

            }


        if(PIDVariant_ == "PID"){

            integral_sum = ki_ * dt * error; 
            ctrld_output = kp_ * error + kd_ * (error - prev_error)/dt + ki_ * (prev_integral_sum + integral_sum) + input;

            if(hasLimt_){

                if(ctrld_output >= UpperLimt_) output = UpperLimt_;
                if(ctrld_output <= LowerLimt_) output = LowerLimt_;
                else output = ctrld_output;
                prev_integral_sum = integral_sum;
                prev_error = error;

            }

            else{

                output = ctrld_output;
                prev_integral_sum = integral_sum;
                prev_error = error;

            }
        }

        end_time = start_time;

    }

}


float controllers::PID::getOutput(){

        return output;

}