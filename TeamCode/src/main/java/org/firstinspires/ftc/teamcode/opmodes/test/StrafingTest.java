package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class StrafingTest extends LinearOpMode {
    //    Pose2d START_POSE = new Pose2d(0, 0, 0);
    Drivetrain drivetrain;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private ElapsedTime timer;
    private Robot robot;

    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        Drivetrain drivetrain;
        robot = new Robot(telemetry, hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        double power = 1;
        while (!isStopRequested() && opModeIsActive()) {
            if (timer.seconds() < 11 && timer.seconds() > 10) {
                robot.drive.setMotorPowers(-1,1,-1,1);
//                drivetrain.leftFront.setPower(1 * power);
//                drivetrain.rightFront.setPower(-1 * power);
//                drivetrain.leftRear.setPower(-1 * power);
//                drivetrain.rightRear.setPower(1 * power);
            } else {
                robot.drive.setMotorPowers(0,0,0,0);
//                drivetrain.leftFront.setPower(0);
//                drivetrain.rightFront.setPower(0);
//                drivetrain.rightRear.setPower(0);
//                drivetrain.leftRear.setPower(0);
            }
            telemetry.addData("time", timer);
            telemetry.addData("Velocity Left Front", drivetrain.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Velocity Right Front", drivetrain.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Velocity Left Rear", drivetrain.leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Velocity Right Rear", drivetrain.rightRear.getCurrent(CurrentUnit.AMPS));

            telemetry.update();

        }
    }
}