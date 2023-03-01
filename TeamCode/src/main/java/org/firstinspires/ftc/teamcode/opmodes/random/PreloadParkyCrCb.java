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
public class PreloadParkyCrCb extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-64,Math.toRadians(180));
    Pose2d Preload_POSE = new Pose2d(35,-9.5, Math.toRadians(180.5));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
    private ElapsedTime timer;

    public double liftHigh = 1075;
    public double liftMid = 800;
    public double liftLow = 600;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 630;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 1.0;
    public double hzslidesin = 0.3;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.closeClaw();
        sleep(1500);
        robot.intake.dropArm();
        detector.init(hardwareMap, telemetry);

        robot.turret.MAX_POWER = 0.615;
        robot.drive.voltagemode = "auto";

        TrajectorySequence preloadBlueMiddle = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(hzslidesin);

                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.turret.setTargetAngle(503);
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(0.73);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(910);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                    robot.turret.setTargetAngle(640);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(160);
                })
                .waitSeconds(0.5)
                .strafeLeft(24)
                .waitSeconds(0.5)
                .back(24)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                    robot.lift.setTargetHeight(0);
                })
                .build();

        TrajectorySequence preloadMagentaLeft = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(hzslidesin);

                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.turret.setTargetAngle(503);
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(0.73);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(910);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                    robot.turret.setTargetAngle(640);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(160);
                })
                .waitSeconds(0.5)
                .strafeLeft(24)
                .waitSeconds(0.5)
                .forward(24)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                    robot.lift.setTargetHeight(0);
                })
                .build();

        TrajectorySequence preloadRedRight = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    robot.intake.centerArm();
                    robot.lift.setTargetHeight(340);
                    robot.lift.setHorizontalPosition(hzslidesin);

                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.turret.setTargetAngle(503);
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(0.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    robot.lift.setHorizontalPosition(0.73);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(910);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                    robot.turret.setTargetAngle(0);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(160);
                })
                .waitSeconds(0.5)
                .strafeLeft(24)
                .addTemporalMarker(() -> {
                    robot.intake.closeClaw();
                    robot.lift.setTargetHeight(0);
                })
                .build();

        robot.drive.setPoseEstimate(START_POSE);

        SleeveDetectionPipeline.Color Parking = detector.getColor();

        waitForStart();

        if (Parking == SleeveDetectionPipeline.Color.RED) {
            robot.drive.followTrajectorySequenceAsync(preloadRedRight);
        } else if (Parking == SleeveDetectionPipeline.Color.MAGENTA) {
            robot.drive.followTrajectorySequenceAsync(preloadMagentaLeft);
        } else {
            robot.drive.followTrajectorySequenceAsync(preloadBlueMiddle);
        }

        //robot.drive.followTrajectorySequenceAsync(cycleLow);

        while (opModeIsActive()) {
            //Pose2d poseEstimate = robot.drive.getPoseEstimate();
            //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
//            telemetry.addData("turret target", robot.turret.getTargetAngle());
//            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            robot.update();
            //drive.update();
        }
    }
}