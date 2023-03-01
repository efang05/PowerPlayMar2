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
public class OdoDriveTest extends LinearOpMode{
    private Robot robot;
    private final double dtRate = 0.5;
    private SlewRateLimiter SRL;
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    FtcDashboard dashboard;


    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        robot.init();

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        TelemetryPacket packet = new TelemetryPacket();
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        waitForStart();
        robot.drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.lift.setArmPos(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        while(!isStopRequested()) {
            telemetry.addData("turret goal", robot.turret.getTargetAngle());
            telemetry.addData("encoder left value: ", leftEncoder.getCurrentPosition());
            telemetry.addData("encoder right value: ", rightEncoder.getCurrentPosition());
            telemetry.addData("encoder front value: ", frontEncoder.getCurrentPosition());

            packet.put("encoder left value: ", leftEncoder.getCurrentPosition());
            packet.put("encoder right value: ", rightEncoder.getCurrentPosition());
            packet.put("encoder front value: ", frontEncoder.getCurrentPosition());
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