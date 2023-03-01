package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret implements Subsystem {

    public DcMotorEx tmotor;

    public double kP = 0.012, kI = 0, kD = 0.000002, ff = 0;
    public double integralSum = 0;
    private double lastError = 0;

    public double targetAngle = 0;
    private double currentAngle;
    private final double deadzone = 2;
    public double rotation = 0;

    private final double ticks_to_degrees = 1.0;
    public double MAX_POWER = 1.0;

    private double PID;

    private double testPower;

    ElapsedTime timer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        tmotor = hardwareMap.get(DcMotorEx.class, "turret");
    }

    @Override
    public void init() {
        setTargetAngle(0);
        tmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        PID = PIDController(targetAngle, currentAngle);

        currentAngle = getCurrentAngle();

        if (Math.abs(currentAngle - targetAngle) < deadzone) {
            setTurretPower(0);
        } else {
            setTurretPower(PIDController(targetAngle, currentAngle));
        }
    }

    @Override
    public void update(TelemetryPacket packet) {
    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double getCurrentAngle() {
        double angle = tmotor.getCurrentPosition() * ticks_to_degrees;
        return angle;
    }

    public void setTurretPower(double power) {
        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        testPower = power;
        tmotor.setPower(power);
    }

    public void setRotation(double gamepadInput) {
        rotation = gamepadInput;
    }

    public double getPID() { return PID; }

    public double getMotorPower() { return tmotor.getPower(); }

    public void setFF(double input) {
        ff = input;
    }

    public void setPID (double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }

    public double getTestPower() {return testPower; }

}
