#ifndef PID_H
#define PID_H

// PID class definition
class PID {
public:
    PID(float kp, float ki, float kd, float dt);
    float compute(float setpoint, float measured);
    void reset();

private:
    float _kp, _ki, _kd, _dt;
    float _prevError;
    float _integral;
};

// Init and update hooks for main loop
void initPID();
void updatePID();

#endif


