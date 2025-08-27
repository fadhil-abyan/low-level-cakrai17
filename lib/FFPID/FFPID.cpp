#include "FFPID.h"

FFPID::FFPID(float timeSampling, float kp, float ki, float kd) : 
    MiniPID(kp, ki*timeSampling, timeSampling != 0.0f ? kd/timeSampling : kd) 
{
    this->omega = 0.0f;
    this->rho_input = 0.0f;
    this->dt = timeSampling;

    MiniPID::reset();
}

void FFPID::changePIDParam(float kp, float ki, float kd) {
    if (this->dt == 0.0f) {
        this->setPID(kp, ki, kd);
    }
    else {
        this->setPID(kp, ki, kd/this->dt);
    }
}

float FFPID::getOmega() {
    return this->omega;
}

float FFPID::getOutput_velControl(float sensorSpeed, float target, float maxVoltageAbs) {
    this->rho_input = (MiniPID::getOutput(sensorSpeed, target) + 8.2358 * sensorSpeed + tanhf(sensorSpeed * 0.7) * 94.9057) / (-0.228 * fabs(sensorSpeed) + 32.9737);
    this->rho_input = MiniPID::clamp(this->rho_input, -maxVoltageAbs, maxVoltageAbs);
    return this->rho_input/maxVoltageAbs; // Normalize to [-1, 1] for pwm duty cycle output
}

float FFPID::getOutput_posControl(float sensorPos, float sensorSpeed, float target, float maxVoltageAbs) {
    this->rho_input = (MiniPID::getOutput(sensorPos, target) + 8.2358 * sensorSpeed + tanhf(sensorSpeed * 40) * 94.9057)/ (-0.228 * fabs(sensorSpeed) + 32.9737);
    this->rho_input = MiniPID::clamp(this->rho_input, -maxVoltageAbs, maxVoltageAbs);
    return this->rho_input/maxVoltageAbs; // Normalize to [-1, 1] for pwm duty cycle output
}
 
float FFPID::getOutput_miniPID(float sensor, float target, float maxOutput) {
    this->rho_input = MiniPID::getOutput(sensor, target);
    this->rho_input = MiniPID::clamp(this->rho_input, -maxOutput, maxOutput);
    return this->rho_input; // Normalize to [-1, 1] for pwm duty cycle output
}

MiniPID::MiniPID(float p, float i, float d) {
    init();
    P = p; I = i; D = d;
}

MiniPID::MiniPID(float p, float i, float d, float f) {
    init();
    P = p; I = i; D = d; F = f;
}

void MiniPID::init() {
    P = 0.0f;
    I = 0.0f;
    D = 0.0f;
    F = 0.0f;

    maxIOutput = 0.0f;
    maxError = 0.0f;
    errorSum = 0.0f;
    maxOutput = 0.0f; 
    minOutput = 0.0f;
    setpoint = 0.0f;
    lastActual = 0.0f;
    firstRun = true;
    reversed = false;
    outputRampRate = 0.0f;
    lastOutput = 0.0f;
    outputFilter = 0.0f;
    setpointRange = 0.0f;
}

void MiniPID::setP(float p) { P = p; checkSigns(); }
void MiniPID::setI(float i) { 
    if(I != 0.0f) errorSum = errorSum * I / i;
    if(maxIOutput != 0.0f) maxError = maxIOutput / i;
    I = i; 
    checkSigns(); 
}
void MiniPID::setD(float d) { D = d; checkSigns(); }
void MiniPID::setF(float f) { F = f; checkSigns(); }
void MiniPID::setPID(float p, float i, float d) { P = p; I = i; D = d; checkSigns(); }
void MiniPID::setPID(float p, float i, float d, float f) { P = p; I = i; D = d; F = f; checkSigns(); }

void MiniPID::setMaxIOutput(float maximum) {
    maxIOutput = maximum;
    if(I != 0.0f) maxError = maxIOutput / I;
}

void MiniPID::setOutputLimits(float output) { setOutputLimits(-output, output); }
void MiniPID::setOutputLimits(float minimum, float maximum) {
    if(maximum < minimum) return;
    maxOutput = maximum;
    minOutput = minimum;
    if(maxIOutput == 0.0f || maxIOutput > (maximum - minimum)) {
        setMaxIOutput(maximum - minimum);
    }
}

void MiniPID::setDirection(bool reversed) { this->reversed = reversed; }
void MiniPID::setSetpoint(float setpoint) { this->setpoint = setpoint; }
void MiniPID::reset() { firstRun = true; errorSum = 0.0f; }
void MiniPID::setOutputRampRate(float rate) { outputRampRate = rate; }
void MiniPID::setSetpointRange(float range) { setpointRange = range; }

void MiniPID::setOutputFilter(float strength) {
    if(strength == 0.0f || bounded(strength, 0.0f, 1.0f)) {
        outputFilter = strength;
    }
}

float MiniPID::getOutput(float actual, float setpoint) {
    this->setpoint = setpoint;
    
    if(setpointRange != 0.0f) {
        setpoint = clamp(setpoint, actual - setpointRange, actual + setpointRange);
    }

    float error = setpoint - actual;
    float Foutput = F * setpoint;
    float Poutput = P * error;

    if(firstRun) {
        lastActual = actual;
        lastOutput = Poutput + Foutput;
        firstRun = false;
    }

    float Doutput = -D * (actual - lastActual);
    lastActual = actual;

    float Ioutput = I * errorSum;
    if(maxIOutput != 0.0f) Ioutput = clamp(Ioutput, -maxIOutput, maxIOutput);

    float output = Foutput + Poutput + Ioutput + Doutput;

    if(minOutput != maxOutput && !bounded(output, minOutput, maxOutput)) {
        errorSum = error;
    }
    else if(outputRampRate != 0.0f && !bounded(output, lastOutput - outputRampRate, lastOutput + outputRampRate)) {
        errorSum = error;
    }
    else if(maxIOutput != 0.0f) {
        errorSum = clamp(errorSum + error, -maxError, maxError);
    }
    else {
        errorSum += error;
    }

    if(outputRampRate != 0.0f) {
        output = clamp(output, lastOutput - outputRampRate, lastOutput + outputRampRate);
    }
    if(minOutput != maxOutput) {
        output = clamp(output, minOutput, maxOutput);
    }
    if(outputFilter != 0.0f) {
        output = lastOutput * outputFilter + output * (1.0f - outputFilter);
    }

    lastOutput = output;
    return output;
}

float MiniPID::getOutput() { return getOutput(lastActual, setpoint); }
float MiniPID::getOutput(float actual) { return getOutput(actual, setpoint); }

float MiniPID::clamp(float value, float min, float max) {
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

bool MiniPID::bounded(float value, float min, float max) {
    return (min < value) && (value < max);
}

void MiniPID::checkSigns() {
    if(reversed) {
        if(P > 0.0f) P *= -1.0f;
        if(I > 0.0f) I *= -1.0f;
        if(D > 0.0f) D *= -1.0f;
        if(F > 0.0f) F *= -1.0f;
    }
    else {
        if(P < 0.0f) P *= -1.0f;
        if(I < 0.0f) I *= -1.0f;
        if(D < 0.0f) D *= -1.0f;
        if(F < 0.0f) F *= -1.0f;
    }
}