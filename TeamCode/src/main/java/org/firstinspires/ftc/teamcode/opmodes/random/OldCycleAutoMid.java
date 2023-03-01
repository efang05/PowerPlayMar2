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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@Autonomous
public class OldCycleAutoMid extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-67,Math.toRadians(0));
    Pose2d Preintake = new Pose2d(38,-12, Math.toRadians(0));
    Pose2d Preload_POSE = new Pose2d(38,-24, Math.toRadians(0));
    Pose2d Score_POSE = new Pose2d(38,-12, Math.toRadians(0));
    Pose2d Intake_POSE = new Pose2d(56, -12, Math.toRadians(0));
    Pose2d FinalIntake_POSE = new Pose2d(57, -12, Math.toRadians(0));
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
    public double hzslidesout = 1.0;
    public double hzslidesin = 0.3;

    public double scorearm = 0.52;

    public double scoreangle = 165;
    public double preloadangle = 0;
    public double intakeangle = 650;

    private double prev_time = System.currentTimeMillis();

    private double fifthcone = 110;
    private double fourthcone = 65;
    private double thirdcone = 30;
    private double secondcone = 5;
    private double firstcone = 0;
    private double preloadhorizontal = 0.55;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.closeClaw();
        sleep(1500);
        robot.lift.setHorizontalPosition(0.3);
        sleep(1500);
        robot.intake.centerArm();
        detector.init(hardwareMap, telemetry);

        robot.turret.MAX_POWER = 0.85;
        robot.drive.voltagemode = "auto";

        TrajectorySequence Sequence = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.lift.setHorizontalPosition(preloadhorizontal);
                    robot.lift.setTargetHeight(liftHigh);
                    robot.turret.setTargetAngle(preloadangle);
                    robot.intake.setArmPos(scorearm);
                })
                .waitSeconds(0.1)
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
                .lineToLinearHeading(Preintake)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                //Cycle 1
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
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
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                //Cycle 2
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
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
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                //Cycle 3
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
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
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                //Cycle 4
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(FinalIntake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
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
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                //Cycle 5
                .setVelConstraint(robot.drive.getVelocityConstraint(60, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(FinalIntake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, () -> {
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
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Score_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.turret.setTargetAngle(scoreangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh-dropvalue);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.intake.centerArm();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(preintakepos);
                    robot.intake.centerArm();
                })
                .build();

        robot.drive.setPoseEstimate(START_POSE);

        SleeveDetectionPipeline.Color Parking = detector.getColor();

        waitForStart();

        if (Parking.equals(SleeveDetectionPipeline.Color.RED)) {
            Park_POSE = new Pose2d(60,-12, Math.toRadians(180));
        } else if (Parking.equals(SleeveDetectionPipeline.Color.MAGENTA)) {
            Park_POSE = new Pose2d(12,-12, Math.toRadians(180));
        } else {
            Park_POSE = new Pose2d(36,-12, Math.toRadians(180));
        }

        robot.drive.followTrajectorySequenceAsync(Sequence);
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