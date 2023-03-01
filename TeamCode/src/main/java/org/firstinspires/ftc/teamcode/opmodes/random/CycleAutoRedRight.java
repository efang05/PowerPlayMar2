package org.firstinspires.ftc.teamcode.opmodes.random;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;

@Autonomous
public class CycleAutoRedRight extends LinearOpMode {
    private double startingx = 33;
    private double startingy = -64;
    private double startingheading = Math.toRadians(180);
    Pose2d START_POSE = new Pose2d(startingx, startingy, startingheading);
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
    private ElapsedTime timer;

    public double liftHigh = 1150;
    public double liftMid = 700;
    public double liftLow = 350;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 660;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 1.3;
    public double hzslidesin = 0.3;



    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.closeClaw();
        sleep(1500);
        robot.intake.dropArm();
        detector.init(hardwareMap, telemetry);
        Drivetrain drive = new Drivetrain(hardwareMap);
        robot.turret.MAX_POWER = 0.5;
        robot.drive.voltagemode = "teleop";
        TrajectorySequence preload = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))

                // Preplaced

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.intake.centerArm();
                })
                .lineToLinearHeading(new Pose2d(33,-8, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.turret.setTargetAngle(-115);
                    robot.intake.setArmPos(0.55);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> {
                    robot.intake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                    robot.turret.setTargetAngle(turretBack);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(2.3, () -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .build();

        TrajectorySequence Cycling = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .setReversed(false)
                .waitSeconds(2.5)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setTargetHeight(7.5);
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(60, -5, Math.toRadians(180)))
                //pick up cone
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.intake.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftLow);
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.turret.setTargetAngle(575);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftLow);
                    robot.intake.fullyOpenClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.turret.setTargetAngle(turretBack);
                })
                .waitSeconds(0.3)
                .build();


        robot.drive.setPoseEstimate(START_POSE);

        waitForStart();

        robot.drive.followTrajectorySequenceAsync(preload);
        //relocalize();
        robot.drive.followTrajectorySequenceAsync(Cycling);
        //relocalize();
        robot.drive.followTrajectorySequenceAsync(Cycling);


        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret target", robot.turret.getTargetAngle());
//            telemetry.addData("opmode", robot.lift.getOpmode());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            robot.update();
            drive.update();
        }
    }

    private void relocalize() {
        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        double xpos = startingx + poseEstimate.getX();
        double ypos = startingy + poseEstimate.getY();
        double headingpos = startingheading + poseEstimate.getHeading();
        robot.drive.setPoseEstimate(new Pose2d(xpos, ypos, headingpos));
        startingx = xpos;
        startingy = ypos;
        startingheading = headingpos;
    }
}