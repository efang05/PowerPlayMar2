package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.threeten.bp.DayOfWeek;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake implements Subsystem {

    //Arm constants
    public double liftedArm = 0.7;
    public double droppedArm = 0.21;
    public double centeredArm = 0.6;

    //Claw constants
    private double openedClaw = 0.55;
    private double closedClaw = 0.22;
    private double fullyOpen = 0.6;

    public Servo clawServoB;
    public Servo clawServo;
    public Servo armServo1;
    public Servo armServo2;
    public Servo aligner;
//    public CRServo intakeServo1;
//    public CRServo intakeServo2;
//    public CRServo intakeServo3;
//    public CRServo intakeServo4;

    private ElapsedTime colortimer;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServoB = hardwareMap.get(Servo.class, "claw2");
        armServo1 = hardwareMap.get(Servo.class, "arm");
        armServo2 = hardwareMap.get(Servo.class, "arm2");
        aligner = hardwareMap.get(Servo.class, "fallencone");
//        intakeServo1 = hardwareMap.get(CRServo.class, "intake1");
//        intakeServo2 = hardwareMap.get(CRServo.class, "intake2");
//        intakeServo3 = hardwareMap.get(CRServo.class, "intake3");
//        intakeServo4 = hardwareMap.get(CRServo.class, "intake4");
    }

    @Override
    public void init() {
        colortimer = new ElapsedTime();
    }

    @Override
    public void update() {

    }

    @Override
    public void update(TelemetryPacket packet) {

    }

    public void setAligner(double position){
        aligner.setPosition(position);
    }

    public void setClawPos(double position){
        clawServo.setPosition(position);
        clawServoB.setPosition(1-position);
    }

    public void openClaw(){
        setClawPos(openedClaw);
    }

    public void closeClaw() {
        setClawPos(closedClaw);
    }

    public void fullyOpenClaw() {
        setClawPos(fullyOpen);
    }

    public void setArmPos(double position){
        armServo1.setPosition(position);
        armServo2.setPosition(1-position);
    }

    public void liftArm(){
        setArmPos(liftedArm);
    }

    public void intakeArm(){
        setArmPos(0.4);
    }

    public void centerArm(){
        setArmPos(centeredArm);
    }

    public void dropArm(){
        setArmPos(droppedArm);
    }

}