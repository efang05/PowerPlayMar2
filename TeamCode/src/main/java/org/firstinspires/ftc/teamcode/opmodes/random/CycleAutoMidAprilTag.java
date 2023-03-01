package org.firstinspires.ftc.teamcode.opmodes.random;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectorAprilTag;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class CycleAutoMidAprilTag extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-62,Math.toRadians(270));
    Pose2d Preintake = new Pose2d(38,-12, Math.toRadians(0));
    Pose2d Preload_POSE = new Pose2d(38,-12, Math.toRadians(270));
    Pose2d Score_POSE = new Pose2d(41,-12, Math.toRadians(0));
    Pose2d Intake_POSE = new Pose2d(57.75, -12, Math.toRadians(0));
    Pose2d FinalIntake_POSE = new Pose2d(58.75, -12, Math.toRadians(0));
    Robot robot;
    SleeveDetectorAprilTag detector = new SleeveDetectorAprilTag();
    int parkingPos = 100;
    private ElapsedTime timer;

    public double liftHigh = 1170;
    public double liftMid = 900;
    public double liftLow = 350;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;
    public double dropvalue = 300;
    public double preintakepos = 300;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 630;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 0.969;
    public double hzslidesin = 0.3;

    public double scorearm = 0.52;
    private double preloadarm = 0.52;

    public double scoreangle = -538;
    public double preloadangle = -187.5;
    public double intakeangle = 0;

    private double prev_time = System.currentTimeMillis();

    private double fifthcone = 90;
    private double fourthcone = 70;
    private double thirdcone = 50;
    private double secondcone = 30;
    private double firstcone = 10;
    private double preloadhorizontal = 0.72;

    private double intakingturret = 0.8;
    private double outtakingturret = 0.5;
    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.setAligner(0.0);
        robot.intake.closeClaw();
        sleep(1500);
        robot.lift.setHorizontalPosition(0.3);
        sleep(1500);
        robot.intake.centerArm();
        detector.init(hardwareMap, telemetry);

        robot.turret.MAX_POWER = 0.6;
        robot.lift.MIN_POWER = -0.4;
        robot.drive.voltagemode = "auto";

        TrajectorySequence Sequence1 = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(preloadarm);
                    robot.turret.setTargetAngle(preloadangle);
                })
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(preloadhorizontal);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.MAX_POWER = intakingturret;
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(fifthcone);
                    robot.intake.centerArm();
                })
                .turn(Math.toRadians(90))
                //Cycle 1
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.8500000, () -> {
                    robot.lift.setTargetHeight(fifthcone);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.turret.MAX_POWER = outtakingturret;
                    robot.intake.setArmPos(scorearm);
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.MAX_POWER = intakingturret;
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(fourthcone);
                    robot.intake.centerArm();
                })
                //Cycle 2
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.8500000, () -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(fourthcone);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.turret.MAX_POWER = outtakingturret;
                    robot.intake.setArmPos(scorearm);
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.MAX_POWER = intakingturret;
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(thirdcone);
                    robot.intake.centerArm();
                })
                //Cycle 3
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.8500000, () -> {
                    robot.lift.setTargetHeight(thirdcone);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.turret.MAX_POWER = outtakingturret;
                    robot.intake.setArmPos(scorearm);
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.MAX_POWER = intakingturret;
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(secondcone);
                    robot.intake.centerArm();
                })
                //Cycle 4
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(FinalIntake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.8500000, () -> {
                    robot.intake.setArmPos(0.45);
                    robot.lift.setTargetHeight(secondcone);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.turret.MAX_POWER = outtakingturret;
                    robot.intake.setArmPos(scorearm);
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.turret.MAX_POWER = intakingturret;
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(firstcone);
                    robot.intake.centerArm();
                })
                //Cycle 5
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(FinalIntake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.8500000, () -> {
                    robot.lift.setTargetHeight(firstcone);
                    robot.intake.setArmPos(0.45);
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.turret.MAX_POWER = outtakingturret;
                    robot.intake.setArmPos(scorearm);
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                })
//                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
//                .forward(20)
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    robot.lift.setTargetHeight(0);
                })
                .build();

        TrajectorySequence leftPark = robot.drive.trajectorySequenceBuilder(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(12,-12, Math.toRadians(0)))
                .build();

        TrajectorySequence midPark = robot.drive.trajectorySequenceBuilder(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence rightPark = robot.drive.trajectorySequenceBuilder(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
                .build();
        // ==================================================================================

        robot.drive.setPoseEstimate(START_POSE);

        while(!isStarted()){
            parkingPos = detector.getParkPos();

            telemetry.addData("tag", parkingPos);
            telemetry.addData("time", timer.seconds());
            telemetry.update();
        }

        detector.stop();
        // ========================================================================================
        waitForStart();
        robot.drive.followTrajectorySequenceAsync(Sequence1);
        waitAsync();

        if (parkingPos == 99) {
            robot.drive.followTrajectorySequenceAsync(leftPark);
        } else if (parkingPos == 100) {
            robot.drive.followTrajectorySequenceAsync(midPark);
        } else {
            robot.drive.followTrajectorySequenceAsync(rightPark);
        }

        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret target", robot.turret.getTargetAngle());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            robot.update();
        }
    }

    private void waitAsync() {
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {

            double loop_time = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addLine("=============    STATUS    =============");
            telemetry.addData("time", timer.seconds());
            telemetry.addData("loop time", loop_time);
            telemetry.addLine("=============  DRIVE TRAIN  =============");
            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addLine("=============   SUBSYSTEM   =============");
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret target", robot.turret.getTargetAngle());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());

            telemetry.update();
            robot.update();
        }
    }
}