package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class HeadingLock {

    //PID Stuff
    private double kP = 0.0, kI = 0.0, kD = 0.0;
    public static double integralSum = 0;
    private double lastError = 0;

    //Measurements
    public double targetAngle = 0;
    private double angle;
    private double angle_offset = 0;

    private final double MAX_POWER = 1;

    private double PID = 0;
    private double turnPower = 0;

    ElapsedTime PIDTimer = new ElapsedTime();

    BNO055IMU imu;

    public HeadingLock(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public void init() {
        setTargetAngle(0);
    }

    public void update() {
        angle = imu.getAngularOrientation().firstAngle - angle_offset;

        PID = PIDController(targetAngle, angle);

        if (Math.abs(targetAngle - angle) < 2) {
            turnPower = 0;
        } else {
            setTurn(PID);
        }
    }

    public void resetAngle() {
        angle_offset = angle;
    }

    public void setTurn(double power) {
        turnPower = Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * PIDTimer.seconds();
        double derivative = (error - lastError) / PIDTimer.seconds();
        lastError = error;

        PIDTimer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public Double getTargetAngle() {
        return targetAngle;
    }


    public double getCurrentAngle() {
        return angle;
    }

    public double getPID() {
        return PID;
    }

    public double getTurn() {
        return turnPower;
    }

    public void setPID (double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }

}
