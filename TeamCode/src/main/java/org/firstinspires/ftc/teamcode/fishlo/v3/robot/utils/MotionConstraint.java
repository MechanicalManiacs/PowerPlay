package org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils;

public class MotionConstraint {
    public double max_acceleration;

    public double max_decceleration;

    public double max_velocity;

    public MotionConstraint(double max_acceleration, double max_decceleration, double max_velocity) {
        this.max_acceleration = max_acceleration;
        this.max_decceleration = max_decceleration;
        this.max_velocity = max_velocity;
    }
}
