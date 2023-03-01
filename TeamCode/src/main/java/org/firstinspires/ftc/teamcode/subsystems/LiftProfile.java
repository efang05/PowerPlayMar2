package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.controls.ProfiledPID;

public class LiftProfile implements Subsystem {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private static double kP = 0.022, kI = 0, kD = 0.0003; //0.6, 0, 0.01
    private static double kF = 0.16;

    private static int MAX_VEL;
    private static int MAX_ACCEL;

    public static double integralSum = 0;
    private double lastError = 0;

    private double targetHeight = 0;
    private double currentHeight;

    private final double ticks_to_inches = 1.0;
    private final double MAX_POWER = 1;

    private double PID;

    private ProfiledPID controller;

    ElapsedTime PIDTimer = new ElapsedTime();

    ElapsedTime mpTimer = new ElapsedTime();

    public LiftProfile(HardwareMap hardwareMap, Telemetry telemetry) {
        motor1 = hardwareMap.get(DcMotorEx.class, "lift1");
        motor2 = hardwareMap.get(DcMotorEx.class, "lift2");

        //reverse correctly
        motor1.setDirection(DcMotorEx.Direction.REVERSE);

        controller = new ProfiledPID(kP, kD, kI, kF, MAX_VEL, MAX_ACCEL);
    }

    @Override
    public void init() {
        setTargetHeight(0);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        //PID = PIDController(targetHeight, currentHeight);

        if (targetHeight > 1000) {
            targetHeight = 1000;
        }
        currentHeight = getCurrentHeight();

        if (targetHeight == 0 && currentHeight < 0.3) {
            setLiftPower(0);
        } else if (targetHeight > currentHeight && motor1.getCurrent(CurrentUnit.AMPS) > 10) {
            setTargetHeight(0);
        } else {
            setLiftPower(controller.calculate(targetHeight));
        }
    }

    @Override
    public void update(TelemetryPacket packet) {

    }

    public void setLiftPower(double power) {
        power = Range.clip(power, -0.3,MAX_POWER);
        motor2.setPower(power);
        motor1.setPower(power);
    }


    public void setTargetHeight(double height) {
        targetHeight = height;
        controller.setGoal(targetHeight);
    }

    public Double getTargetHeight() {
        return targetHeight;
    }


    public double getCurrentHeight() {
        double height = motor2.getCurrentPosition() * ticks_to_inches;
        return height;
    }

    public double getMotorPower() {
        return motor2.getPower();
    }

    public double getPID() {
        return PID;
    }

    public void setPIDF (double P, double I, double D, double F) {
        kP = P;
        kI = I;
        kD = D;
        kF = F;
    }
}
