// Obtained from Leonard Cross https://www.instructables.com/Upside-Up-Robot-Balancing-Revisited/

// PID Controller Class
class PropIntDiff {

    public:                 
        volatile double ITerm, lastInput, dInput, Output;
        volatile double kp, ki, kd;
        volatile double error, MIN, MAX, ITermThreshold;
        bool inAuto;    

        PropIntDiff(double MINlimit, double MAXlimit, double Threshold);
        void initialize();
        void calculate(double Setpoint, double Input);
        void calculate(double Setpoint, double Input, double dInput);
};