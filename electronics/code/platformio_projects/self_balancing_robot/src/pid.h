// Obtained from Leonard Cross https://www.instructables.com/Upside-Up-Robot-Balancing-Revisited/

// PID Controller Class
class PropIntDiff {

    public:                 
        volatile float ITerm, lastInput, dInput, Output;
        volatile float kp, ki, kd;
        volatile float error, MIN, MAX, ITermThreshold;
        bool inAuto;    

        PropIntDiff(float MINlimit, float MAXlimit, float Threshold);
        void initialize();
        void calculate(float Setpoint, float Input);
        void calculate(float Setpoint, float Input, float dInput);
};