package org.firstinspires.ftc.teamcode.opmodes.random;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.controls.SlewRateLimiter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class SlidesManualTest extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    private SlewRateLimiter SRL;
    private final double slideRate = 0.5;

    public enum Height {
        DOWN,
        LOW,
        MID,
        HIGH
    }

    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        SRL = new SlewRateLimiter(slideRate);
        robot.init();

        waitForStart();
        //robot.lift.setHeight(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        Height height = Height.DOWN;
        timer.reset();
        while(!isStopRequested()) {
            telemetry.addData("slide height", robot.lift.getCurrentHeight());
            telemetry.addData("slide target", robot.lift.getTargetHeight());
            telemetry.addData("slide power", robot.lift.getPID());
            telemetry.addData("gamepad input", gamepad2.left_stick_y);
            telemetry.addData("gamepad input", SRL.calculate(gamepad1.left_stick_y));

            robot.lift.setTargetHeight(robot.lift.getCurrentHeight() + SRL.calculate(gamepad1.left_stick_y) * 0.5);

            robot.update();
            telemetry.update();

        }
    }

}