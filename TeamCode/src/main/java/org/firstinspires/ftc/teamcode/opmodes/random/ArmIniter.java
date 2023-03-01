package org.firstinspires.ftc.teamcode.opmodes.random;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ArmIniter extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    private double prev_time = System.currentTimeMillis();
    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();

        robot.init();

        robot.intake.setArmPos(0.5);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addData("loop time", dt);
            }
            telemetry.update();
            robot.update();
        }
    }

