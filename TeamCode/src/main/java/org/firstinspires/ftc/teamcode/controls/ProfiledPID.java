package org.firstinspires.ftc.teamcode.controls;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProfiledPID {
    private PIDController m_controller;
    private MotionState m_goal;
    private MotionState m_setpoint;
    private MotionProfile profile;
    private double MAX_VEL, MAX_ACCEL, MAX_JERK;
    private ElapsedTime timer = new ElapsedTime();

    public ProfiledPID(double Kp, double Ki, double Kd, double Kf, double vel, double accel) {
        m_controller = new PIDController(Kp, Ki, Kd, Kf);
        MAX_VEL = vel;
        MAX_ACCEL = accel;
        MAX_JERK = 100;
    }

    public ProfiledPID(double Kp, double Ki, double Kd, double Kf, double vel, double accel, double jerk) {
        m_controller = new PIDController(Kp, Ki, Kd, Kf);
        MAX_VEL = vel;
        MAX_ACCEL = accel;
        MAX_JERK = jerk;
    }

    public void setPID(double Kp, double Ki, double Kd) {
        m_controller.setPID(Kp, Ki, Kd);
    }

    public void setGoal(double goal) {
        m_goal = new MotionState(goal, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(m_setpoint, m_goal, MAX_VEL, MAX_ACCEL, MAX_JERK);
        timer.reset();
    }

    public boolean atSetpoint() {
        return m_controller.atSetPoint();
    }

    public boolean atGoal() {
        return atSetpoint() && m_goal.equals(m_setpoint);
    }

    public MotionState getGoal() {
        return m_goal;
    }

    public void setConstraints(double vel, double accel) {
        MAX_VEL = vel;
        MAX_ACCEL = accel;
    }

    public MotionState getSetpoint() {
        return m_setpoint;
    }

    public double getPeriod() {
        return m_controller.getPeriod();
    }

    public double calculate(double measurement) {
        m_setpoint = profile.get(timer.seconds());
        return m_controller.calculate(measurement, m_setpoint.getX());
    }

}
