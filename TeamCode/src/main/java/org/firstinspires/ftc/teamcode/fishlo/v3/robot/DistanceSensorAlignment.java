package org.firstinspires.ftc.teamcode.fishlo.v3.robot;

import com.arcrobotics.ftclib.hardware.SensorDistanceEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class DistanceSensorAlignment extends SubSystem {

    SensorDistanceEx distanceSensor;
    Map<String, SensorDistanceEx.DistanceTarget> targets;

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public DistanceSensorAlignment(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        distanceSensor = robot.hardwareMap.get(SensorDistanceEx.class, "distanceSensor");
        targets = new LinkedHashMap<>();
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public void addTarget(double target, String name) {
        SensorDistanceEx.DistanceTarget t = new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, target, 0.5, name);
        targets.put(name, t);
        distanceSensor.addTarget(t);
    }

    public boolean isTargetReached(String name) {
        return distanceSensor.targetReached(targets.get(name));
    }
}
