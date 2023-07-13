#if !defined(CONTROLLER_H)
#define CONTROLLER_H

#include <Arduino.h>

class controller_interface{


    virtual float getOutput() = 0;


};


namespace controllers{


    class PID : public controller_interface{

        public:
            PID(float &kp, float &kd, float &ki, int &controller_freq, String &type, bool &hasLimt);
            PID();
            void Setkp(float &kp);
            void Setkd(float &kd);
            void Setki(float &ki);
            void SetCtrlr_freq(int &freq);
            void SetPIDVariant(String &type);
            void computeOutput(float &nput, float &setPoint);
            float getOutput() override;
            void setLimits(double &Upperlimt, double &LowerLimt);

        private:
            float output;
            float kp_;
            float kd_;
            float ki_;
            double UpperLimt_;
            double LowerLimt_;
            float error;
            bool hasLimt_;
            float prev_error;
            float integral;
            float prev_integral_sum;
            float integral_sum;
            int controller_freq_;
            double dt = 1.0f/double(controller_freq_);
            float start_time;
            float end_time;
            String PIDVariant_; 


    };




}

#endif // CONTROLLER_H
