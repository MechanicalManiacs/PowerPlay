package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class InspectionAuto extends FishloAutonomousProgram {

    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    VisionPipeline.ConePosition conePosition;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = drive.getInstance();
        lift.resetEncoder();
        startPose = new Pose2d(-72, 36, Math.toRadians(180));
        mecanumDrive.setPoseEstimate(startPose);
        telemetry.addLine("Initialized");
        telemetry.update();
        telemetry.setAutoClear(true);
    }

    @Override
    public void main() {
//        Trajecto
//        mecanumDrive.followTrajectorySequence(sequence);
    }
}

