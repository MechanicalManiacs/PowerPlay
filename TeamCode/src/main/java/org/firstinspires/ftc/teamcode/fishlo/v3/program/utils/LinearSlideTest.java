package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v3.robot.LinearSlide;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class LinearSlideTest extends FishloAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("Start");
        telemetry.update();
    }

    @Override
    public void main() {
        lift.liftSlide(LinearSlide.Level.HIGH, 1);
        sleep(30000);
    }
}
