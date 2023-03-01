package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@TeleOp
public class SlidesPIDTest extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    public enum Height {
        DOWN,
        LOW,
        MID,
        HIGH
    }

    FtcDashboard dashboard;

    public static double TARGET_POS = 0;

    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        //robot.lift.setHeight(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        Height height = Height.DOWN;
        timer.reset();

        while(!isStopRequested()) {
            telemetry.addData("slide height", robot.lift.getCurrentHeight());
            telemetry.addData("lift PID", robot.lift.getPID());

            robot.update();
            telemetry.update();

            switch (height) {
                case DOWN:
                    robot.lift.setTargetHeight(0);
                    if (timer.milliseconds() > 1500) {
                        height = Height.LOW;
                        timer.reset();
                    }
                    break;
                case LOW:
                    robot.lift.setTargetHeight(250);
                    if (timer.milliseconds() > 1500) {
                        height = Height.MID;
                        timer.reset();
                    }
                    break;
                case MID:
                    robot.lift.setTargetHeight(500);
                    if (timer.milliseconds() > 2500) {
                        height = Height.HIGH;
                        timer.reset();
                    }
                    break;
                case HIGH:
                    robot.lift.setTargetHeight(800);
                    if (timer.milliseconds() > 1500) {
                        height = Height.LOW;
                        timer.reset();
                    }
                    break;
            }
        }
    }
}