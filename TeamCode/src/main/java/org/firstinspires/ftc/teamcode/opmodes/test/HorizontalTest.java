package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.controls.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class HorizontalTest extends LinearOpMode{
    private Robot robot;
    FtcDashboard dashboard;


    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        robot.init();

        TelemetryPacket packet = new TelemetryPacket();
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        //robot.lift.setArmPos(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        while(!isStopRequested()) {
            if (gamepad1.a) {
                robot.lift.horizontalServo1.setPosition(1);
            } else if (gamepad1.b) {
                robot.lift.horizontalServo1.setPosition(0);
            } else if (gamepad1.x) {
                robot.lift.horizontalServo2.setPosition(1);
            } else if (gamepad1.y) {
                robot.lift.horizontalServo2.setPosition(0);
            }

            dashboard.sendTelemetryPacket(packet);

            robot.update();
            telemetry.update();
            robot.turret.setRotation(gamepad1.left_stick_x);

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y, //controls forward
                            -gamepad1.left_stick_x, //controls strafing
                            -gamepad1.right_stick_x //controls turning
                    )
            );


        }
    }

}