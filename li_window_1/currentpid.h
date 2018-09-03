#ifndef CURRENTPID_H
#define CURRENTPID_H


class CurrentPID
{
private:
    float KP;
    float KI;
    float KD;
    float PreError;
    float LastError;
    float PID_Calculate(float ThisError);

public:
    CurrentPID();
    void SetParameter(float kp, float ki, float kd, float preError, float lastError);
    float Tension_Control(float tensionValue, float nowTension);
};

#endif // CURRENTPID_H
