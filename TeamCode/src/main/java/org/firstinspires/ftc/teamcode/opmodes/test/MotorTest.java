package org.firstinspires.ftc.teamcode.opmodes.test;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class MotorTest extends LinearOpMode{
    Robot robot;

    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        waitForStart();

        while(opModeIsActive()){
            robot.drive.rightRear.setPower(0.5);
            robot.drive.rightFront.setPower(0.5);
            robot.drive.leftRear.setPower(0.5);
            robot.drive.leftFront.setPower(0.5);


            robot.update();
            telemetry.update();
        }
    }
}
