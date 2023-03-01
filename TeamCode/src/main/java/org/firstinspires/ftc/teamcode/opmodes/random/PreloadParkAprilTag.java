package org.firstinspires.ftc.teamcode.opmodes.random;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectorAprilTag;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@Autonomous
public class PreloadParkAprilTag extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-66,Math.toRadians(270));
    Pose2d Preload_POSE = new Pose2d(40,-15, Math.toRadians(270));
    Pose2d Score_POSE = new Pose2d(38,-15, Math.toRadians(270));
    Pose2d Intake_POSE = new Pose2d(54,-15, Math.toRadians(270));
    Pose2d Park_POSE = new Pose2d(36,-15, Math.toRadians(270));
    Robot robot;
    SleeveDetectorAprilTag detector = new SleeveDetectorAprilTag();
    int parkingPos = 100;
    private ElapsedTime timer;

    public double liftHigh = 1320;
    public double liftMid = 850;
    public double liftLow = 500;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;
    public double dropvalue = 75;
    public double preintakepos = 200;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 630;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 1.0;
    public double hzslidesin = 0.3;

    public double scorearm = 0.53;
    public int count = 0;

    public double scoreangle = 480;
    public double preloadangle = 480;

    double prev_time = 0;

    private void waitAsync() {
        while (robot.drive.isBusy() && opModeIsActive() && !isStopRequested()) {

            double loop_time = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addLine("=============    STATUS    =============");
            telemetry.addData("time", timer.seconds());
            telemetry.addData("loop time", loop_time);
            telemetry.addData("tag", parkingPos);
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("=============  DRIVE TRAIN  =============");
            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("=============   SUBSYSTEM   =============");
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret target", robot.turret.getTargetAngle());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());

            telemetry.update();
            robot.update();
        }
    }

    private TrajectorySequence build_preload() {
        TrajectorySequence Sequence = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Preload_POSE)
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.turret.setTargetAngle(533);
//                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(0.73);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(910);
                })
                .waitSeconds(10.0)
//                .addTemporalMarker(() -> {
//                    robot.intake.fullyOpenClaw();
//                })
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> {
//                    robot.lift.setHorizontalPosition(hzslidesin);
//                    robot.intake.centerArm();
//                    robot.turret.setTargetAngle(640);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    robot.lift.setTargetHeight(160);
//                })
//                .waitSeconds(0.5)
//                .strafeLeft(24)
//                .waitSeconds(0.5)
//                .back(24)
//                .addTemporalMarker(() -> {
//                    robot.intake.closeClaw();
//                    robot.lift.setTargetHeight(0);
//                })
                .build();

        return Sequence;
    }

    private void auto_init() {
        detector.init(hardwareMap, telemetry);
        robot = new Robot(telemetry, hardwareMap);
        robot.init();
        robot.intake.closeClaw();
        sleep(500);
        robot.intake.dropArm();

        robot.turret.MAX_POWER = 0.725;
        robot.drive.voltagemode = "auto";
    }
    public void runOpMode() {
        timer = new ElapsedTime();
        String start_position = "right";

        auto_init();

        // =========================================================
        TrajectorySequence Sequence = build_preload();

        TrajectorySequence leftPark = robot.drive.trajectorySequenceBuilder(Preload_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(40,-12, Math.toRadians(270)))
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(12,-12, Math.toRadians(270)))
                .build();

        TrajectorySequence midPark = robot.drive.trajectorySequenceBuilder(Preload_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(40,-12, Math.toRadians(270)))
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(270)))
                .build();

        TrajectorySequence rightPark = robot.drive.trajectorySequenceBuilder(Preload_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(40,-12, Math.toRadians(270)))
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(270)))
                .build();

        // ===========================================================================

        robot.drive.setPoseEstimate(START_POSE);

        while(!isStarted()){
            parkingPos = detector.getParkPos();
            telemetry.addData("tag", parkingPos);
            telemetry.addData("time", timer.seconds());
            telemetry.update();
        }

        waitForStart();
        detector.stop();
        timer.reset();
        // auto sequence preload
        robot.drive.followTrajectorySequenceAsync(Sequence);
        waitAsync();

        if (parkingPos == 99) {
            robot.drive.followTrajectorySequenceAsync(leftPark);
        } else if (parkingPos == 100) {
            robot.drive.followTrajectorySequenceAsync(midPark);
        } else {
            robot.drive.followTrajectorySequenceAsync(rightPark);
        }

        while (opModeIsActive() && !isStopRequested()) {

            double loop_time = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addLine("=============    STATUS    =============");
            telemetry.addData("time", timer.seconds());
            telemetry.addData("loop time", loop_time);
            telemetry.addData("tag", parkingPos);
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