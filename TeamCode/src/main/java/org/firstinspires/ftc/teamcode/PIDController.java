package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    double integralSum = 0;

    public double Basic(double current, double reference, double Tolerance){
        double error = reference - current;
        if(Math.abs(error) >= Tolerance){
            if(error < 0){
                return -1;
            }
            else{
                return 1;
            }
        }
        else{
            return 0;
        }
    }

    public double Calc(double current, double reference, double Kp, double Ki, double Kd, double Tolerance){
        double error = reference - current;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error*timer.seconds();
        timer.reset();
        if(Math.abs(error) >= Tolerance){
            return (Kp*error) + (Kd*derivative) + (Ki*integralSum);
        }
        else{
            return 0;
        }
    }
}
