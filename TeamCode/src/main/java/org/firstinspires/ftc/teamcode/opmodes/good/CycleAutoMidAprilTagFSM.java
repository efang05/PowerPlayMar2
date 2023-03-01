package org.firstinspires.ftc.teamcode.opmodes.good;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectorAprilTag;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class CycleAutoMidAprilTagFSM extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-62,Math.toRadians(270));
    Pose2d Preload_POSE = new Pose2d(38,-11, Math.toRadians(270));
    Pose2d Score_POSE = new Pose2d(43,-12, Math.toRadians(0));
    Pose2d Intake_POSE = new Pose2d(59.25, -12, Math.toRadians(0));
    Pose2d FinalIntake_POSE = new Pose2d(58.75, -12, Math.toRadians(0));
    Robot robot;
    SleeveDetectorAprilTag detector = new SleeveDetectorAprilTag();
    int parkingPos = 100;
    private ElapsedTime timer;

    public double liftHigh = 1175;
    public double liftMid = 900;
    public double liftLow = 350;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;
    public double dropvalue = 300;
    public double preintakepos = 300;
    public int count = 0;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 630;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 0.969;
    public double hzslidesin = 0.3;

    public double scorearm = 0.62;
    private double preloadarm = 0.62;

    public double scoreangle = -523;
    public double preloadangle = -155.5;
    public double intakeangle = 0;

    private double prev_time = System.currentTimeMillis();

    private double coneStack = 160;
    private double coneSubtract = 24;
    private double preloadhorizontal = 1.0;

    private double intakingturret = 0.8;
    private double outtakingturret = 0.5;

    private double scorealigner = 0.61;

    public enum State {
        PRELOAD,
        INTAKE,
        SCORE,
        PARKING,
        IDLE
    };

    State currentState = State.PRELOAD;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.setAligner(0.0);
        robot.intake.closeClaw();
        sleep(500);
        robot.lift.setHorizontalPosition(0.3);
        sleep(500);
        robot.intake.centerArm();
        detector.init(hardwareMap, telemetry);

        robot.turret.MAX_POWER = 0.6;
        robot.lift.MIN_POWER = -0.4;
        robot.drive.voltagemode = "auto";

        // START_POSE -> preload -> turn -> Score_POSE
        TrajectorySequence preload = build_preload();
        // Score_POSE -> Intake_POSE
        TrajectorySequence intake = build_intake(Score_POSE);
        // Intake_POSE -> Score_POSE
        TrajectorySequence[] cycle = new TrajectorySequence[5];
        for(count = 0; count < 5; count++) {
                cycle[count] = build_cycle(Intake_POSE, count);
        }
        // Score_POSE -> Parking
        TrajectorySequence leftPark = robot.drive.trajectorySequenceBuilder(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(8,-12, Math.toRadians(0)))
                .build();

        TrajectorySequence midPark = robot.drive.trajectorySequenceBuilder(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence rightPark = robot.drive.trajectorySequenceBuilder(Score_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(64,-12, Math.toRadians(0)))
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
        robot.drive.followTrajectorySequenceAsync(preload);
        //waitAsync();

        int cycleCnt = 0;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PRELOAD:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectorySequenceAsync(intake);
                        currentState = State.INTAKE;
                    }
                    break;
                case INTAKE:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectorySequenceAsync(cycle[cycleCnt]);
                        currentState = State.SCORE;
                    }
                    break;
                case SCORE:
                    if (!robot.drive.isBusy()) {
                        cycleCnt++;
                        if (cycleCnt >= 5) {
                            // Finished 5 cycles
                            if (parkingPos == 99) {
                                robot.drive.followTrajectorySequenceAsync(leftPark);
                            } else if (parkingPos == 100) {
                                robot.drive.followTrajectorySequenceAsync(midPark);
                            } else {
                                robot.drive.followTrajectorySequenceAsync(rightPark);
                            }
                            currentState = State.PARKING;
                        } else {
                            // Continue cycling
                            robot.drive.followTrajectorySequenceAsync(intake);
                            currentState = State.INTAKE;
                        }
                    }
                    break;
                case PARKING:
                    if (!robot.drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("cycle number", cycleCnt);
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            robot.update();
        }
    }

    private TrajectorySequence build_preload() {
        TrajectorySequence preload = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                    robot.lift.setTargetHeight(liftHigh);
                    robot.intake.setArmPos(preloadarm);
                    robot.turret.setTargetAngle(preloadangle);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.intake.setAligner(scorealigner);
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
                    robot.intake.setAligner(0);
                    robot.turret.MAX_POWER = intakingturret;
                    robot.turret.setTargetAngle(intakeangle);
                    robot.lift.setTargetHeight(coneStack);
                    robot.intake.centerArm();
                })
                .turn(Math.toRadians(90))
                .lineToLinearHeading(Score_POSE)
                .build();
        return preload;
    }

    private TrajectorySequence build_intake(Pose2d start_pose) {
        Pose2d Test_Pose = new Pose2d(43,-11.6, Math.toRadians(0)); // Manually ffing the pose
        TrajectorySequence intake = robot.drive.trajectorySequenceBuilder(start_pose) //replace with start_pose for normal
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(Intake_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    robot.intake.closeClaw();
                })
                .build();

        return intake;
    }

    private TrajectorySequence build_cycle(Pose2d start_pose, int count) {
        TrajectorySequence cycle = robot.drive.trajectorySequenceBuilder(start_pose)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(43,-11, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-1.1, () -> {
                    robot.turret.MAX_POWER = outtakingturret;
                    robot.intake.setArmPos(scorearm);
                    robot.turret.setTargetAngle(scoreangle);
                    robot.intake.setAligner(scorealigner);
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
                    robot.intake.setAligner(0);
                    robot.turret.setTargetAngle(intakeangle);
                    setCycleIntakeHeight(count);
                })
                .build();
        return cycle;
    }

    private void setCycleIntakeHeight(int count) {
        if (count == 0) {
            robot.lift.setTargetHeight(160);
            robot.intake.centerArm();
        } else if (count == 1) {
            robot.lift.setTargetHeight(96);
            robot.intake.centerArm();
        } else if (count == 2) {
            robot.lift.setTargetHeight(60);
            robot.intake.centerArm();
        } else if (count == 3) {
            robot.lift.setTargetHeight(37);
            robot.intake.setArmPos(0.55);
        } else if (count == 4) {
            robot.lift.setTargetHeight(30);
            robot.intake.setArmPos(0.55);
        }
    }

}