package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class TurretPIDTest2 extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;

    FtcDashboard dashboard;

    public static int TARGET_POS = 0;

    public static double kP = 1, kI = 0, kD = 0, ff = 0.3;

    private double prev_time = System.currentTimeMillis();

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        TelemetryPacket packet = new TelemetryPacket();

        MultipleTelemetry tele = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        //robot.lift.setHeight(LiftConstants.IdleArm);
        tele.addData("loop:", "started");
        timer.reset();

        while (!isStopRequested()) {
            robot.lift.setTargetHeight(650);

            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            tele.addData("turret goal", robot.turret.getTargetAngle());
            tele.addData("turret pos", robot.turret.getCurrentAngle());
            tele.addData("turret PID", robot.turret.getPID());
            tele.addData("slide height", robot.lift.getCurrentHeight());
            tele.addData("tmotor power", robot.turret.getMotorPower());
            tele.addData("rotation", -gamepad1.right_stick_x);
            tele.addData("testPower", robot.turret.getTestPower());

            packet.put("turret goal", robot.turret.getTargetAngle());
            packet.put("turret pos", robot.turret.getCurrentAngle());
            packet.put("turret PID", robot.turret.getPID());
            packet.put("slide height", robot.lift.getCurrentHeight());
            packet.put("tmotor power", robot.turret.getMotorPower());
            packet.put("rotation", -gamepad1.right_stick_x);
            packet.put("testPower", robot.turret.getTestPower());


            dashboard.sendTelemetryPacket(packet);

            robot.update();
            telemetry.update();

            robot.turret.setTargetAngle(TARGET_POS);
            robot.turret.setFF(ff);
            robot.turret.setPID(kP, kI, kD);

            robot.turret.setRotation(-gamepad1.right_stick_x);

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