//Imports
package org.firstinspires.ftc.teamcode.opmodes.good;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

// mason was here
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    //Constants
    private Robot robot;
    private robotState robotState;
    private fallenState fallenState;
    private robotMode robotMode;
    private switchNormal switchNormal;
    private switchStack switchStack;
    private ElapsedTime timer;
    private ElapsedTime turrettimer;
    private ElapsedTime armtimer;
    private ElapsedTime autotimer;
    private ElapsedTime timer2;
    private ElapsedTime timer3;
    private boolean canTurn = false;
    double turretaddition = 10;
    double dtspeed = 1;
    public double up = 1700;
    public double mid = 1250;
    public double low = 800;
    public double ground = 0;
    public double idle = 450;
    public double intaking = -5;
    private double front = 0;
    private double back = 660;
    private double right = 330;
    private double left = -330;
    private double droppedvalue = 150;
    private boolean armup = true;
    private boolean lowheight = false;
    private double horizontalback = 0.51;
    private double horizontallifted = 0.5;
    private double horizontalmiddle = 0.5;
    private double horizontalextended = 1;
    private String horizontalpos = "back";
    private String liftedpos = "lifted";
    private boolean running = false;
    private double highposarm = 0.6;
    private int wowie = 0;

    private double prev_time = System.currentTimeMillis();

    public enum robotState {
        IDLE,
        IDLE2,
        INTAKING,
        GRABBED,
        LIFTED,
        DROPPED
    }

    public enum robotMode {
        STACK,
        NORMAL,
    }

    public enum switchNormal {
        Idle,
        FirstActions,
        SecondActions
    }
    public enum switchStack {
        Idle,
        FirstActions,
        SecondActions
    }

    public enum fallenState {
        up,
        down
    }

    public void runOpMode() throws InterruptedException {
//Init
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        turrettimer = new ElapsedTime();
        armtimer = new ElapsedTime();
        autotimer = new ElapsedTime();
        timer2 = new ElapsedTime();
        timer3 = new ElapsedTime();
        waitForStart();
        robot.init();
        robotState = robotState.INTAKING;
        robot.intake.openClaw();
        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.turret.tmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armup = true;
        robot.turret.MAX_POWER = 1;
        robot.lift.MIN_POWER = -0.9;
        robotMode = robotMode.NORMAL;
        fallenState = fallenState.up;
        switchNormal = switchNormal.Idle;
        switchStack = switchStack.Idle;
        robot.lift.setHorizontalPosition(horizontalback);
        robot.drive.voltagemode = "teleop";
        robot.intake.setAligner(0);
        double TurretPower = gamepad2.right_stick_x;


//Drivetrain
        while (!isStopRequested() && opModeIsActive()) {

            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addData("loop time", dt);


            //anti-tip + regular teleop code - ONLY ON PITCH RIGHT NOW
            double antiTipMulti = 1;
//            double correctedpitch = robot.drive.getOrientation().thirdAngle + 3.13;
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * dtspeed, //  controls forward
                            -gamepad1.left_stick_x * dtspeed, +//controls strafing
                            -gamepad1.right_stick_x * dtspeed //controls turning
                    )
            );

            //Slow-Mode
            if (gamepad1.right_bumper == true) {
                dtspeed = 0.4;
            } else {
                dtspeed = 1;
            }

            //Turret Manual Control
            if (canTurn = true) {
                if (TurretPower > 0.5) {
                    robot.turret.setTargetAngle(robot.turret.getCurrentAngle() - turretaddition);
                } else if (TurretPower < -0.5) {
                    robot.turret.setTargetAngle(robot.turret.getCurrentAngle() + turretaddition);
                }
            }

            switch(fallenState){
                case up:
                    robot.intake.setAligner(0);
                    if (timer3.milliseconds()>300 && gamepad2.left_bumper && robotState == robotState.INTAKING){
                        timer3.reset();
                        fallenState = fallenState.down;
                    }
                    break;
                case down:
                    robot.intake.setAligner(0.485);
                    if (timer3.milliseconds()>300 && gamepad2.left_bumper && robotState == robotState.INTAKING){
                        timer3.reset();
                        fallenState = fallenState.up;
                    }
                    break;
            }

            switch (robotMode) {
                case NORMAL:
                    if (-gamepad2.left_stick_y > 0.5) {
                        robot.lift.targetHeight += 8;
                    } else if (-gamepad2.left_stick_y < -0.5) {
                        robot.lift.targetHeight -= 8;
                    }

                    if (gamepad2.right_stick_x > 0.5) {
                        robot.turret.targetAngle -= 5;
                    } else if (gamepad2.right_stick_x < -0.5) {
                        robot.turret.targetAngle += 5;
                    }
                    //Turret Presets
                    if (canTurn = true) {
                        if (turrettimer.milliseconds() > 500) {
                            if (gamepad2.a) {
                                robot.turret.setTargetAngle(back);
                                turrettimer.reset();
                            }
                            if (gamepad2.x) {
                                robot.turret.setTargetAngle(right);
                                turrettimer.reset();
                            }
                            if (gamepad2.b && gamepad2.start == false) {
                                robot.turret.setTargetAngle(left);
                                turrettimer.reset();
                            }
                            if (gamepad2.y) {
                                robot.turret.setTargetAngle(front);
                                turrettimer.reset();
                            }
                            if (gamepad1.y) {
                                robot.turret.setTargetAngle(front);
                                turrettimer.reset();
                            }
                        }
                    }

                    //FSM
                    switch (robotState) {
                        case IDLE:
                            canTurn = true;
                            robot.intake.liftArm();
                            robot.lift.setHorizontalPosition(horizontallifted);
                            robot.lift.setTargetHeight(idle);
                            if (timer.milliseconds() > 300) {
                                robot.intake.closeClaw();
                                if (gamepad1.left_bumper || timer.milliseconds() > 1250) {
                                    robot.intake.intakeArm();
                                    timer.reset();
                                    robotState = robotState.IDLE2;
                                }
                            }
                            break;
                        case IDLE2:
                            if (timer.milliseconds() > 300) {
                                robot.intake.setAligner(0);
                                robot.lift.setHorizontalPosition(horizontalback);
                                robot.lift.setTargetHeight(intaking);
                                robotState = robotState.INTAKING;
                                timer.reset();
                                timer2.reset();
                            }
                            break;
                        case INTAKING:
                            canTurn = false;
                            horizontalpos = "back";
                            robot.intake.openClaw();
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper) {
                                    robot.intake.closeClaw();
                                    robotState = robotState.GRABBED;
                                    timer2.reset();
                                    timer.reset();
                                }
                            }
                            break;
                        case GRABBED:
                            canTurn = true;
                            robot.intake.setAligner(0.075);
                            robot.lift.setHorizontalPosition(horizontallifted);
                            if (timer.milliseconds() > 300) {
                                robot.intake.liftArm();
                                robot.lift.setTargetHeight(idle);
                            }
                            if (timer.milliseconds() > 200) {
                                if (gamepad1.left_bumper) {
                                    robot.intake.intakeArm();
                                    timer.reset();
                                    robotState = robotState.IDLE2;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    robot.lift.setHorizontalPosition(horizontalmiddle);
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                    robot.lift.setHorizontalPosition(horizontalmiddle);
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                    robot.lift.setHorizontalPosition(horizontalmiddle);
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_down) {
                                    robot.lift.setTargetHeight(ground);
                                    robot.intake.intakeArm();
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = false;
                                    lowheight = false;
                                }
                            }
                            break;
                        case LIFTED:
                            canTurn = true;
                            dtspeed = 0.4;
                            horizontalpos = "middle";
                            liftedpos = "lifted";
                            if (timer.milliseconds() > 750) {
                                if (gamepad2.dpad_down || gamepad1.dpad_down) {
                                    timer.reset();
                                    robot.turret.setTargetAngle(front);
                                    robot.lift.setTargetHeight(idle);
                                    robotState = robotState.GRABBED;
                                }
                                if (gamepad1.left_bumper) {
                                    if (liftedpos == "lifted") {
                                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
                                    }
                                    timer.reset();
                                    robotState = robotState.DROPPED;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    timer.reset();
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                }
                            }
                            if (gamepad2.right_bumper && horizontalpos == "middle") {
                                robot.lift.setHorizontalPosition(horizontalextended);
                                horizontalpos = "extended";
                            } else if (gamepad2.right_bumper && horizontalpos == "extended") {
                                robot.lift.setHorizontalPosition(horizontalmiddle);
                                horizontalpos = "middle";
                            }
                            break;
                        case DROPPED:
                            dtspeed = 1;
                            canTurn = false;
                            horizontalpos = "back";
                            robot.intake.openClaw();
                            if (timer.milliseconds() > 450) {
                                double marker3 = timer.milliseconds() + 300;
                                if (lowheight == false) {
                                    robot.lift.setTargetHeight(idle);
                                    robot.intake.liftArm();
                                    robot.turret.setTargetAngle(front);
                                    robotState = robotState.IDLE;
                                    timer.reset();
                                } else {
                                    robot.lift.setTargetHeight(idle);
                                    robot.intake.liftArm();
                                    if (timer.milliseconds() > marker3) {
                                        robot.turret.setTargetAngle(front);
                                        robotState = robotState.IDLE;
                                        timer.reset();
                                    }
                                }

                            }
                            break;

                    }
                    break;


                case STACK:
                    //Switching Modes
//                    switch(switchToAUTOCYCLE) {
//                        case Idle:
//                                if (gamepad1.x) {
//                                    switchToAUTOCYCLE = switchToAUTOCYCLE.FirstActions;
//                                    timer.reset();
//                                }
//                            break;
//                        case FirstActions:
//                            robot.lift.setTargetHeight(idle);
//                            robot.intake.liftArm();
//                            robot.intake.closeClaw();
//                            robot.lift.setHorizontalPosition(horizontallifted);
//                            switchToAUTOCYCLE = switchToAUTOCYCLE.SecondActions;
//                            timer.reset();
//                            break;
//                        case SecondActions:
//                            if (timer.milliseconds() > 700) {
//                                robot.turret.setTargetAngle(front);
//                                switchToAUTOCYCLE = switchToAUTOCYCLE.ThirdActions;
//                                timer.reset();
//                            }
//                            break;
//                        case ThirdActions:
//                            if (timer.milliseconds() > 300) {
//                                robot.lift.setTargetHeight(intaking);
//                                robot.intake.centerArm();
//                                robot.intake.fullyOpenClaw();
//                                robotMode = robotMode.AUTOCYCLE;
//                                switchToAUTOCYCLE = switchToAUTOCYCLE.Idle;
//                                timer.reset();
//                            }
//                            break;
//                    }

                    switch (switchNormal) {
                        case Idle:
                            if (gamepad1.y) {
                                switchNormal = switchNormal.FirstActions;
                                timer.reset();
                            }
                            break;
                        case FirstActions:
                            robot.lift.setTargetHeight(idle);
                            robot.intake.liftArm();
                            robot.intake.closeClaw();
                            robot.lift.setHorizontalPosition(horizontallifted);
                            switchNormal = switchNormal.SecondActions;
                            timer.reset();
                            break;
                        case SecondActions:
                            if (timer.milliseconds() > 700) {
                                robot.turret.setTargetAngle(front);
                                robotMode = robotMode.NORMAL;
                                robotState = robotState.IDLE;
                                switchNormal = switchNormal.Idle;
                                timer.reset();
                            }
                    }


                    //Horizontal Slides Extend/Retract
                    if (robotState == robotState.LIFTED && gamepad2.right_bumper && horizontalpos == "middle") {
                        robot.lift.setHorizontalPosition(horizontalextended);
                        horizontalpos = "extended";
                    } else if (robotState == robotState.LIFTED && gamepad2.right_bumper && horizontalpos == "extended") {
                        robot.lift.setHorizontalPosition(horizontalmiddle);
                        horizontalpos = "middle";
                    }

                    if (-gamepad2.left_stick_y > 0.5) {
                        robot.lift.targetHeight += 3;
                    } else if (-gamepad2.left_stick_y < -0.5) {
                        robot.lift.targetHeight -= 3;
                    }

                    //Vertical Slides SlightDrop/SlightLift
//                    if (robotState == robotState.LIFTED && gamepad2.left_bumper && liftedpos == "lifted") {
//                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
//                        liftedpos = "dropped";
//                    } else if (robotState == robotState.LIFTED && gamepad2.left_bumper && liftedpos == "dropped") {
//                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() + droppedvalue);
//                        liftedpos = "lifted";
//                    }

                    //Turret Presets
                    if (canTurn = true) {
                        if (turrettimer.milliseconds() > 500) {
                            if (gamepad2.a) {
                                robot.turret.setTargetAngle(back);
                                turrettimer.reset();
                            }
                            if (gamepad2.x) {
                                robot.turret.setTargetAngle(right);
                                turrettimer.reset();
                            }
                            if (gamepad2.b && gamepad2.start == false) {
                                robot.turret.setTargetAngle(back - left);
                                turrettimer.reset();
                            }
                            if (gamepad2.y) {
                                robot.turret.setTargetAngle(front);
                                turrettimer.reset();
                            }
                            if (gamepad2.a && gamepad2.start == false) {
                                robot.turret.setTargetAngle(back);
                                turrettimer.reset();
                            }
                        }
                    }

                    //FSM
                    switch (robotState) {
                        case IDLE:
                            canTurn = true;
                            robot.intake.closeClaw();
                            robot.intake.liftArm();
                            robot.lift.setHorizontalPosition(horizontallifted);
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper) {
                                    robot.lift.setTargetHeight(50);
                                    robot.lift.setHorizontalPosition(0.6);
                                    timer.reset();
                                    robotState = robotState.INTAKING;
                                }
                            }
                            break;
                        case INTAKING:
                            canTurn = false;
                            robot.intake.dropArm();
                            robot.intake.openClaw();
                            robot.intake.fullyOpenClaw();
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper) {
                                    robot.intake.closeClaw();
                                    robotState = robotState.GRABBED;
                                    timer.reset();
                                }
                            }
                            break;
                        case GRABBED:
                            canTurn = true;
                            robot.lift.setHorizontalPosition(horizontallifted);
                            if (timer.milliseconds() > 300) {
                                robot.intake.liftArm();
                                robot.lift.setTargetHeight(idle);
                            }
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper) {
                                    robot.lift.setHorizontalPosition(0.6);
                                    robot.lift.setTargetHeight(50);
                                    timer.reset();
                                    timer2.reset();
                                    robotState = robotState.INTAKING;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    robot.lift.setHorizontalPosition(0.64);
                                    liftedpos = "lifted";
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                    robot.lift.setHorizontalPosition(horizontalmiddle);
                                    liftedpos = "lifted";
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                    robot.lift.setHorizontalPosition(horizontalmiddle);
                                    liftedpos = "lifted";
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_down) {
                                    if (Math.abs(robot.turret.getCurrentAngle() - back) < 5) {
                                        robot.lift.setTargetHeight(ground);
                                        robot.intake.dropArm();
                                        timer.reset();
                                        robotState = robotState.LIFTED;
                                        armup = false;
                                        lowheight = true;
                                    } else {
                                        robot.turret.setTargetAngle(back);
                                        if (Math.abs(robot.turret.getCurrentAngle() - back) < 5) {
                                            robot.lift.setTargetHeight(ground);
                                            robot.intake.dropArm();
                                            timer.reset();
                                            robotState = robotState.LIFTED;
                                            armup = false;
                                            lowheight = true;
                                        }
                                    }
                                }
                            }
                            break;
                        case LIFTED:
                            canTurn = true;
                            dtspeed = 0.4;
                            horizontalpos = "middle";
                            if (timer.milliseconds() > 750) {
                                if (gamepad2.dpad_down || gamepad1.dpad_down) {
                                    timer.reset();
                                    robot.turret.setTargetAngle(back);
                                    robot.lift.setTargetHeight(idle);
                                    robotState = robotState.GRABBED;
                                }
                                if (gamepad1.left_bumper) {
                                    robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
                                    timer.reset();
                                    robotState = robotState.DROPPED;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    timer.reset();
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                }
                            }
                            break;
                        case DROPPED:
                            dtspeed = 1;
                            canTurn = false;
                            robot.intake.openClaw();
                            if (timer.milliseconds() > 450) {
                                double marker2 = timer.milliseconds() + 300;
                                robot.lift.setTargetHeight(idle);
                                robot.intake.liftArm();
                                robot.turret.setTargetAngle(back);
                                robot.lift.setTargetHeight(idle);
                                robotState = robotState.IDLE;
                                timer.reset();
                                robot.lift.setTargetHeight(idle);
                                robot.intake.liftArm();
                                if (timer.milliseconds() > marker2) {
                                    robot.turret.setTargetAngle(back);
                                    robot.lift.setTargetHeight(idle);
                                    robotState = robotState.IDLE;
                                    timer.reset();
                                }
                            }
                            break;

                    }
                    break;
            }

            double firstAngle = robot.drive.getOrientation().firstAngle;
            double secondAngle = robot.drive.getOrientation().secondAngle;
            double thirdAngle = robot.drive.getOrientation().thirdAngle;


            telemetry.addData("first angle", firstAngle);
            telemetry.addData("second angle", secondAngle);
            telemetry.addData("third angle", thirdAngle);
            robot.update();
        }
    }

    public void autoAlign() {

    }

}