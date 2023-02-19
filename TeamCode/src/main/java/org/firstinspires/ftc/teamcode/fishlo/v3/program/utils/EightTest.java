package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class EightTest extends FishloAutonomousProgram {
    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    final int OFFSET = 7;
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = drive.getInstance();
        startPose = new Pose2d(-72+OFFSET, 36, 0);
    }

    @Override
    public void main() {
        mecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence sequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-48, 12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48+24, 12+24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48+24+24, 12+24-24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48+24+24+24, 12+24-24+24), Math.toRadians(0))
                .build();
        mecanumDrive.followTrajectorySequence(sequence);
    }
}
