#ifndef FFPID_H
#define FFPID_H
#include <cmath>

class MiniPID {
public:
    MiniPID(float, float, float);
    MiniPID(float, float, float, float);
    void setP(float);
    void setI(float);
    void setD(float);
    void setF(float);
    void setPID(float, float, float);
    void setPID(float, float, float, float);
    void setMaxIOutput(float);
    void setOutputLimits(float);
    void setOutputLimits(float, float);
    void setDirection(bool);
    void setSetpoint(float);
    void reset();
    void setOutputRampRate(float);
    void setSetpointRange(float);
    void setOutputFilter(float);
    float getOutput();
    float getOutput(float);
    float getOutput(float, float);

protected:
    float clamp(float, float, float);
    bool bounded(float, float, float);
    void checkSigns();
    void init();
    float P;
    float I;
    float D;
    float F;

    float maxIOutput;
    float maxError;
    float errorSum;

    float maxOutput; 
    float minOutput;

    float setpoint;

    float lastActual;

    bool firstRun;
    bool reversed;

    float outputRampRate;
    float lastOutput;

    float outputFilter;

    float setpointRange;
};

class FFPID : public MiniPID {
private:
    float omega;
    float rho_input;
    float dt;
    
public:
    FFPID(float timeSampling, float kp, float ki, float kd);
    
    void changePIDParam(float kp, float ki, float kd);
    float getOmega();
    float getRho() { return this->rho_input; }
    float getOutput_velControl(float sensor, float target, float maxAbsOut);
    float getOutput_posControl(float sensorPos, float sensorSpeed, float target, float maxVoltageAbs);
    float getOutput_miniPID(float sensor, float target, float maxOutput);
};

#endif