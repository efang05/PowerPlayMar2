package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TurretPIDTest extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    private double prev_time = System.currentTimeMillis();
    public enum ArmPos {
        LEFT,
        RIGHT
    }

    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();

        waitForStart();
        //robot.lift.setArmPos(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        ArmPos Pos = ArmPos.LEFT;
        timer.reset();
        while(!isStopRequested()) {
            robot.lift.setTargetHeight(400);

            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addData("loop time", dt);
            telemetry.addData("turret goal", robot.turret.getTargetAngle());
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret power", robot.turret.getPID());
            telemetry.addData("slide height", robot.lift.getCurrentHeight());
            telemetry.addData("tmotor power", robot.turret.getMotorPower());
            telemetry.addData("lift PID", robot.lift.getPID());

//            telemetry.addData("turret mode", robot.lift.getTurretMode());

            robot.update();
            telemetry.update();
//            robot.drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y, //controls forward
//                            -gamepad1.left_stick_x, //controls strafing
//                            -gamepad1.right_stick_x //controls turning
//                    )
//            );
            switch (Pos) {
                case LEFT:
                    robot.turret.setTargetAngle(125);
                    robot.turret.tmotor.setPower(0.5);
                    if (timer.milliseconds() > 2500) {
                        Pos = ArmPos.RIGHT;
                        timer.reset();
                    }
                    break;
                case RIGHT:
                    robot.turret.setTargetAngle(-125);
                    if (timer.milliseconds() > 2500) {
                        Pos = ArmPos.LEFT;
                        timer.reset();
                    }
                    break;
            }
        }
    }

}