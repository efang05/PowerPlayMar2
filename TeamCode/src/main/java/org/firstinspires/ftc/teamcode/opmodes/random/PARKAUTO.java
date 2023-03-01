package org.firstinspires.ftc.teamcode.opmodes.random;

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
public class PARKAUTO extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-62,Math.toRadians(270));
    Pose2d Preintake = new Pose2d(38,-12, Math.toRadians(0));
    Pose2d Preload_POSE = new Pose2d(38,-12, Math.toRadians(270));
    Pose2d Score_POSE = new Pose2d(41,-12, Math.toRadians(0));
    Pose2d Intake_POSE = new Pose2d(57.75, -12, Math.toRadians(0));
    Pose2d FinalIntake_POSE = new Pose2d(58.75, -12, Math.toRadians(0));
    Pose2d Park_POSE = new Pose2d(36,-12, Math.toRadians(0));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
    private ElapsedTime timer;

    public double liftHigh = 900;
    public double liftMid = 700;
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
    public double hzslidesout = 1;
    public double hzslidesin = 0.3;

    public double scorearm = 0.52;
    private double preloadarm = 0.52;


    public double scoreangle = -495;
    public double preloadangle = -162.5;
    public double intakeangle = 0;

    private double prev_time = System.currentTimeMillis();

    private double fifthcone = 165;
    private double fourthcone = 120;
    private double thirdcone = 85;
    private double secondcone = 75;
    private double firstcone = 40;
    private double preloadhorizontal = 0.72;

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
        robot.lift.MIN_POWER = -0.8;
        robot.drive.voltagemode = "auto";

        TrajectorySequence Sequence1 = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(270), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(0);
                    robot.intake.setArmPos(preloadarm);
                    robot.turret.setTargetAngle(0);
                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
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
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                .turn(Math.toRadians(90))
                .lineToLinearHeading(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .forward(20)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .build();


        TrajectorySequence Sequence2 = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(270), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(0);
                    robot.intake.setArmPos(preloadarm);
                    robot.turret.setTargetAngle(0);
                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
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
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                .turn(Math.toRadians(90))
                .lineToLinearHeading(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .back(28)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .build();

        TrajectorySequence Sequence3 = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(270), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(0);
                    robot.intake.setArmPos(preloadarm);
                    robot.turret.setTargetAngle(0);
                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
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
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                .turn(Math.toRadians(90))
                .lineToLinearHeading(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .back(4)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(0);
                })
                .build();

        robot.drive.setPoseEstimate(START_POSE);

        SleeveDetectionPipeline.Color Parking = detector.getColor();

        while (opModeInInit()) {
            Parking = detector.getColor();
            telemetry.addData("VAL:", detector.getColor());
            telemetry.update();
        }

        waitForStart();

        if (Parking.equals(SleeveDetectionPipeline.Color.RED)) {
            robot.drive.followTrajectorySequenceAsync(Sequence1);
        } else if (Parking.equals(SleeveDetectionPipeline.Color.MAGENTA)) {
            robot.drive.followTrajectorySequenceAsync(Sequence2);
        } else{
            robot.drive.followTrajectorySequenceAsync(Sequence3);
        }

        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addData("loop time", dt);
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
            //drive.update();
        }
    }
}