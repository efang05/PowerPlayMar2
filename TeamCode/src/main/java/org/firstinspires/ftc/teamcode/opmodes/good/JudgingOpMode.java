package org.firstinspires.ftc.teamcode.opmodes.good;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class JudgingOpMode extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    private robotState robotState;

    public enum robotState {
        IDLE,
        EXTEND,
        RETURN,
        TURN,
        EXTEND2,
        RETURN2
    }

    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();

        robot.intake.setArmPos(0.4);
        robot.intake.fullyOpenClaw();
        robot.lift.setHorizontalPosition(0.3);
        robot.lift.setTargetHeight(0);

        waitForStart();

        robotState state = robotState.IDLE;

        while(!isStopRequested() && opModeIsActive()) {
            switch (state) {
                case IDLE:
                    robot.intake.setArmPos(0.4);
                    robot.intake.fullyOpenClaw();
                    robot.lift.setHorizontalPosition(0.3);
                    robot.lift.setTargetHeight(0);
                    if (gamepad1.a) {
                        robot.lift.setHorizontalPosition(1.0);
                        state = robotState.EXTEND;
                        timer.reset();
                    }
                    break;
                case EXTEND:
                    robot.intake.closeClaw();
                    if (timer.seconds() > 0.5) {
                        robot.lift.setTargetHeight(250);
                        state = robotState.RETURN;
                        timer.reset();
                    } else if (timer.seconds() > 0.3) {
                        robot.lift.setHorizontalPosition(0.3);
                        robot.intake.liftArm();
                    }
                    break;
                case RETURN:
                    if (timer.seconds() > 0.3) {
                        robot.turret.setTargetAngle(660);
                        robot.lift.setTargetHeight(650);
                        state = robotState.TURN;
                        timer.reset();
                    }
                    break;
                case TURN:
                    if (timer.seconds() > 0.4) {
                        robot.lift.setHorizontalPosition(1.0);
                        robot.intake.fullyOpenClaw();
                        state = robotState.EXTEND2;
                        timer.reset();
                    }
                    break;
                case EXTEND2:
                    if (timer.seconds() > 0.6) {
                        robot.turret.setTargetAngle(0);
                        state = robotState.RETURN2;
                        timer.reset();
                    } else if (timer.seconds() > 0.3) {
                        robot.lift.setHorizontalPosition(0.3);
                    }
                    break;
                case RETURN2:
                    if (timer.seconds() > 0.2) {
                        robot.lift.setTargetHeight(0);
                        state = robotState.IDLE;
                    }
                    break;
            }
            telemetry.addData("State", state);
            telemetry.update();
            robot.update();
        }
    }

}