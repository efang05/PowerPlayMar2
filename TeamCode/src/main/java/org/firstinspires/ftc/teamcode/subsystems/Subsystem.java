package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public interface Subsystem {

    void init();

    void update();

    void update(TelemetryPacket packet);
}