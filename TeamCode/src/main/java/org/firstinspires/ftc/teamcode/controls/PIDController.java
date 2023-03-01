package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double measuredValue;

    private static double kP, kI, kD, kF;
    private static double integralSum;
    private double lastError;

    private double errorVal;
    private double errorTolerance = 0.05;

    private double lastTimeStamp;
    private double period;
    private ElapsedTime timer;

    public PIDController(double kP, double kI, double kD, double kF) {
        kP = kP;
        kI = kI;
        kD = kD;
        kF = kF;
        integralSum = 0;
        lastError = 0;

        lastTimeStamp = 0;
        period = 0;

        timer = new ElapsedTime();
    }

    public double calculate(double setpoint, double measuredValue) {
        errorVal = setpoint - measuredValue;

        double currentTimeStamp = timer.seconds();

        if (lastTimeStamp == 0) {
            lastTimeStamp = currentTimeStamp;
        }

        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        integralSum += errorVal * period;
        double derivative = (errorVal - lastError) / period;
        lastError = errorVal;

        double output = (errorVal * kP) + (derivative * kD) + (integralSum * kI) + kF;
        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTimeStamp = 0;
    }

    public void setTolerance(double positionTolerance) {
        errorTolerance = positionTolerance;
    }

    public boolean atSetPoint() {
        return Math.abs(errorVal) < errorTolerance;
    }

    public void setPID(double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }

    public double[] getCoefficients() {
        return new double[]{kP, kI, kD};
    }

    public double getPeriod() {
        return period;
    }
}
