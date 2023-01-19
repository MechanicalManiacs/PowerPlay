package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.LinearSlide;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class LinearSlideTest extends FishloAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    DcMotorEx liftInstance;
    ElapsedTime timer;

    @Override
    public void preMain() {
        telemetry.addLine("START");
        telemetry.update();
    }

    @Override
    public void main() {
        lift.moveAndDrop(LinearSlide.Level.LOW);
    }
}
