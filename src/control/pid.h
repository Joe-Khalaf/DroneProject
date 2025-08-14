#pragma once

class PID {
public:
    PID(float kp_, float ki_, float kd_);
    float compute(float targetAngle, float currentAngle, float currentRate, float dt_sec);
    void  reset();

    // ---- telemetry getters (read-only) ----
    float pTerm()   const { return pTerm_; }
    float iTerm()   const { return iTerm_; }
    float dTerm()   const { return dTerm_; }
    float error()   const { return error_; }
    float output()  const { return output_; }
    float integralState() const { return integral; }

private:
    float kp, ki, kd;
    float integral;

    // stored each compute() for debugging/telemetry
    float pTerm_, iTerm_, dTerm_, error_, output_;
};

// your global instance (rename if using roll)
extern PID pidPitch;

