package org.firstinspires.ftc.teamcode.opmodes.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class OdoTest extends LinearOpMode{
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public void runOpMode(){
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        while(!isStopRequested()){
            telemetry.addData("encoder left value: ", leftEncoder.getCurrentPosition());
            telemetry.addData("encoder right value: ", rightEncoder.getCurrentPosition());
            telemetry.addData("encoder front value: ", frontEncoder.getCurrentPosition());

            telemetry.update();
        }
    }
}