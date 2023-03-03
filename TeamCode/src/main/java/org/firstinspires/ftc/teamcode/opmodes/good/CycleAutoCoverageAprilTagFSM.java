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
public class CycleAutoCoverageAprilTagFSM extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-62,Math.toRadians(270));
    Pose2d Preload_POSE = new Pose2d(38,-48, Math.toRadians(270));
    Pose2d Score_POSE = new Pose2d(38.5,-12, Math.toRadians(0));
    Pose2d Intake_POSE = new Pose2d(58, -12, Math.toRadians(0));
    Pose2d FinalIntake_POSE = new Pose2d(59, -12, Math.toRadians(0));
    Robot robot;
    SleeveDetectorAprilTag detector = new SleeveDetectorAprilTag();
    int parkingPos = 100;
    private ElapsedTime timer;

    public double liftPickUp = 1000;
    public double liftHigh = 1700;
    public double liftMid = 1212;
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
    public double hzslidesmid = 0.800;

    public double scorearm = 0.61;
    private double preloadarm = 0.61;

    public double mid_scoreangle = -493;
    public double high_scoreangle = 493;
    public double low_scoreangle = -360;
    public double preloadangle = -330;
    public double intakeangle = 0;

    private double prev_time = System.currentTimeMillis();

    private double coneStack = 160;
    private double coneSubtract = 24;
    private double preloadhorizontal = 1.0;

    private double intakingturret = 0.8;
    private double outtakingturret = 0.5;

    private double scorealigner = 0.64;

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
        // Preload -> Intake_POSE
        TrajectorySequence intake = build_intake(Score_POSE);
        // Score_POSE -> Intake_POSE
        TrajectorySequence[] intake_cycle = new TrajectorySequence[5];
        for(int cnt = 0; cnt < 5; cnt++) {
            intake_cycle[cnt] = build_intake(Score_POSE, cnt);
        }
        // Intake_POSE -> Score_POSE
        TrajectorySequence[] cycle = new TrajectorySequence[5];
        for(int cnt = 0; cnt < 5; cnt++) {
            cycle[cnt] = build_cycle(Intake_POSE, cnt);
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
                        robot.drive.followTrajectorySequenceAsync(cycle[count]);
                        currentState = State.SCORE;
                    }
                    break;
                case SCORE:
                    if (!robot.drive.isBusy()) {
                        count++;
                        if (count >= 5) {
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
                            robot.drive.followTrajectorySequenceAsync(intake_cycle[count]);
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
            telemetry.addData("cycle number", count);
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
                    robot.lift.setTargetHeight(liftMid);
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
                    robot.lift.setTargetHeight(liftMid-dropvalue);
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

    private TrajectorySequence build_intake(Pose2d start_pose, int num) {
        Pose2d position = new Pose2d();
        if (num >= 3) {
            position = FinalIntake_POSE;
        } else {
            position = Intake_POSE;
        }
        TrajectorySequence intake = robot.drive.trajectorySequenceBuilder(start_pose) //replace with start_pose for normal
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(position)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    robot.intake.closeClaw();
                })
                .build();

        return intake;
    }

    private TrajectorySequence build_cycle(Pose2d start_pose, int num) {
        double scoreangle = 0;
        Pose2d position = new Pose2d();

        if (num == 0) {
            scoreangle = low_scoreangle;
            position = Intake_POSE;
        } else if (num == 1) {
            scoreangle = mid_scoreangle;
            position = Score_POSE;
        } else if (num >= 2) {
            scoreangle = high_scoreangle;
            position = Score_POSE;
        }

        double finalScoreangle = scoreangle;
        TrajectorySequence cycle = robot.drive.trajectorySequenceBuilder(start_pose)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftPickUp);
                    robot.lift.setHorizontalPosition(hzslidesmid);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftMid);
                    robot.intake.setArmPos(scorearm);
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(position)
                .UNSTABLE_addTemporalMarkerOffset(-1.1, () -> {
                    robot.turret.MAX_POWER = outtakingturret;
                    robot.turret.setTargetAngle(finalScoreangle);
                    robot.intake.setAligner(scorealigner);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(liftMid-dropvalue);
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

    private void setCycleIntakeHeight(int num) {
        if (num == 0) {
            robot.lift.setTargetHeight(71);
            robot.intake.centerArm();
        } else if (num == 1) {
            robot.lift.setTargetHeight(51);
            robot.intake.centerArm();
        } else if (num == 2) {
            robot.lift.setTargetHeight(25);
            robot.intake.centerArm();
        } else if (num == 3) {
            robot.lift.setTargetHeight(20);
            robot.intake.setArmPos(0.52);
        } else if (num == 4) {
            robot.lift.setTargetHeight(20);
            robot.intake.setArmPos(0.52);
        }
    }

}