package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Lift lift;
    public Drivetrain drive;
    public Intake intake;
    public Turret turret;
    public HeadingLock HL;


    private ArrayList<Subsystem> subsystems;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        drive = new Drivetrain(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        HL = new HeadingLock(hardwareMap);

        subsystems = new ArrayList<>();
        subsystems.add(drive);
        subsystems.add(lift);
        subsystems.add(turret);
        subsystems.add(intake);
    }


    //Initializes hardware - NOTE: moves servos
    public void init() {
        for(Subsystem system : subsystems) {
            system.init();
        }
        HL.init();
    }

    public void update() {
        for(Subsystem system : subsystems) {
            system.update();
        }
        HL.update();
        telemetry.update();
    }
}