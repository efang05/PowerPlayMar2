package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
@TeleOp
public class DriveTest extends LinearOpMode{
    private Robot robot;

    public static double kP = 0.0, kI = 0.0, kD = 0.0;
    public double targetAngle = 0.0;
    public double currentAngle = 0.0;

    public enum headingToggle {
        UNLOCKED,
        LOCKED
    }

    FtcDashboard dashboard;


    private double prev_time = System.currentTimeMillis();

    public void runOpMode(){

        robot = new Robot(telemetry, hardwareMap);
        robot.init();

        TelemetryPacket packet = new TelemetryPacket();

        MultipleTelemetry tele = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard = FtcDashboard.getInstance();

        headingToggle Mode = headingToggle.UNLOCKED;

        waitForStart();
        //robot.lift.setArmPos(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        while(!isStopRequested()) {
            robot.HL.setPID(kP, kI, kD);

            double turnPower = robot.HL.getTurn();
            double PID = robot.HL.getPID();

            robot.turret.setTargetAngle(0);
            robot.lift.setTargetHeight(0);
            robot.lift.setHorizontalPosition(0);

            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();
            telemetry.addData("loop time", dt);

            tele.addData("target angle", targetAngle);
            tele.addData("current angle", currentAngle);
            tele.addData("PID", PID);
            tele.addData("turn power", turnPower);
            packet.put("target angle", targetAngle);
            packet.put("current angle", currentAngle);
            packet.put("turn power", turnPower);
            dashboard.sendTelemetryPacket(packet);

            if (gamepad1.a) {
                robot.HL.resetAngle();
            }

            switch (Mode) {
                case UNLOCKED:
                    W_Driving(0);
                    if (gamepad1.left_bumper) {
                        Mode = headingToggle.LOCKED;
                    }
                    break;
                case LOCKED:
                    W_Driving(turnPower);
                    if (gamepad1.right_bumper) {
                        Mode = headingToggle.UNLOCKED;
                    }
            }

            telemetry.update();
            robot.update();
        }
    }

    public void W_Driving(double PID) {
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y, //controls forward
                        -gamepad1.left_stick_x, //controls strafing
                        -(gamepad1.right_stick_x + PID) //controls turning
                )
        );
    }

    public void W_DrivingTwo(double forward, double strafe, double turn, double PID) { // in case i fucked some shit up by accident
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -forward, //controls forward
                        -strafe, //controls strafing
                         -(turn + PID)//controls turning
                )
        );
    }


}