package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.LinearSlide;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class LinearSlideVelocityPIDTuner extends FishloAutonomousProgram {
    public static double targetVel = 0;
    public static double currentVel = 0;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("START");
        telemetry.update();
    }

    @Override
    public void main() {
        while (opModeIsActive()) {
            lift.liftSlide(LinearSlide.Level.HIGH);
            targetVel = lift.getController().getVelocity();
            currentVel = lift.getVelocity();
            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("Current Vel", currentVel);
            telemetry.update();
            lift.update();
        }
    }
}
