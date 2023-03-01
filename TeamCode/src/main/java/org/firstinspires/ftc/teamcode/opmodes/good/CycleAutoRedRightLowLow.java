package org.firstinspires.ftc.teamcode.opmodes.good;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class CycleAutoRedRightLowLow extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(37,-67,Math.toRadians(0));
    Pose2d Preload_POSE = new Pose2d(36,-13, Math.toRadians(0));
    Pose2d Score_POSE = new Pose2d(36,-12, Math.toRadians(0));
    Pose2d Intake_POSE = new Pose2d(54, -12, Math.toRadians(0));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
    private ElapsedTime timer;

    public double liftHigh = 1255;
    public double liftMid = 950;
    public double liftLow = 600;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 630;
    public double turretRight = 990;

    public String park = "none";

    //change after tuning horizontal slides
    public double hzslidesout = 1.0;
    public double hzslidesin = 0.3;
    public double hzslidestransit = 0.48;
    public double hzslidesscore = 0.6012518;

    public double preloadangle = -140;
    public double pipckupangle = -630;
    public double scoreangle = -1020;

    private double prev_time = System.currentTimeMillis();

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.closeClaw();
        sleep(1500);
        robot.lift.setHorizontalPosition(0.3);
        detector.init(hardwareMap, telemetry);

        robot.turret.MAX_POWER = hzslidesscore;
        robot.drive.voltagemode = "auto";

        TrajectorySequence preloadBlueMiddle = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(0.3);

                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.55);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.turret.setTargetAngle(preloadangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(0.73);
                })
                .waitSeconds(0.9)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(850);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                    robot.turret.setTargetAngle(pipckupangle);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(160);
                })
                //Pick Up Cone
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(135);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #1
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(49);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #2
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(30);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #3
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #4
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.setArmPos(0.44);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #5
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(200);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(640);
                    robot.intake.setArmPos(0.52);
                    robot.intake.closeClaw();
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(36,-9.5, Math.toRadians(180.5)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.lift.setTargetHeight(0);
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .build();

        TrajectorySequence preloadMagentaLeft = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(0.3);

                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.54);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.turret.setTargetAngle(preloadangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(0.73);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(930);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                    robot.turret.setTargetAngle(pipckupangle);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(160);
                })
                //Pick Up Cone
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(135);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #1
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(49);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #2
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(30);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #3
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #4
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.setArmPos(0.44);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #5
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(200);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(640);
                    robot.intake.setArmPos(0.52);
                    robot.intake.closeClaw();
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(70, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(16,-9.5, Math.toRadians(180.5)))
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    robot.lift.setTargetHeight(0);
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .build();

        TrajectorySequence preloadRedRight = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(0.3);

                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.54);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.turret.setTargetAngle(preloadangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(0.73);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(930);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                    robot.turret.setTargetAngle(pipckupangle);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(160);
                })
                //Pick Up Cone
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
                    robot.lift.setTargetHeight(135);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #1
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })

                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(49);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #2
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(30);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #3
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #4
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(pipckupangle);
                    robot.intake.setArmPos(0.44);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(380);
                })
                .waitSeconds(0.125)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidestransit);
                })
                .waitSeconds(0.275)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                //Cycle #5
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftLow);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesscore);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(400);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(200);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(640);
                    robot.intake.setArmPos(0.52);
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.3)
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .back(10)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(0);
                })
                .build();

        TrajectorySequence BluePark = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Preload_POSE)
                .lineToLinearHeading(new Pose2d(33.5,-9.5, Math.toRadians(180.5)))
                .build();
        TrajectorySequence MagentaPark = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Preload_POSE)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(14,-9.5, Math.toRadians(180.5)))
                .build();
        TrajectorySequence RedPark = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Preload_POSE)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(53.5,-9.5, Math.toRadians(170)))
                .waitSeconds(1)
                .back(10)
                .build();

        robot.drive.setPoseEstimate(START_POSE);
        SleeveDetectionPipeline.Color Parking = SleeveDetectionPipeline.Color.MAGENTA;

        while (opModeInInit()) {
            Parking = detector.getColor();
            telemetry.addData("VAL:", detector.getColor());
            telemetry.update();
        }

        waitForStart();

        if (Parking.equals(SleeveDetectionPipeline.Color.RED)) {
            park = "red";
            robot.drive.followTrajectorySequenceAsync(preloadRedRight);
//            robot.drive.followTrajectorySequenceAsync(RedPark);
        } else if (Parking.equals(SleeveDetectionPipeline.Color.MAGENTA)) {
            park = "magenta";
            robot.drive.followTrajectorySequenceAsync(preloadMagentaLeft);
//            robot.drive.followTrajectorySequenceAsync(MagentaPark);
        } else {
            park = "blue";
            robot.drive.followTrajectorySequenceAsync(preloadBlueMiddle);
//            robot.drive.followTrajectorySequenceAsync(BluePark);
        }

        //robot.drive.followTrajectorySequenceAsync(cycleLow);

        while (opModeIsActive()) {
            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();
            telemetry.addData("loop time", dt);
            telemetry.addData("horizontal extension", robot.lift.horizontalServo1.getPosition());
            //Pose2d poseEstimate = robot.drive.getPoseEstimate();
            //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
//            telemetry.addData("turret target", robot.turret.getTargetAngle());
//            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("PATH", park);
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            robot.update();
            //drive.update();
        }
    }
}