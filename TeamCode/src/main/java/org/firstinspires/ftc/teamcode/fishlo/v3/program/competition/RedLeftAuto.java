package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class RedLeftAuto extends FishloAutonomousProgram {

    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    final int OFFSET = 7;
    int temporal_marker;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-72, 36, 0);
    }

    @Override
    public void main() {
        mecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence sequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-30,36))
                .splineTo(new Vector2d(-18, 55), Math.toRadians(90))
                //1st CONE
                .lineTo(new Vector2d(-18, 30))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    //move arm
                })
                .lineTo(new Vector2d(-18, 55))

                //2nd CONE
                .lineTo(new Vector2d(-18, 30))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    //move arm
                })
                .lineTo(new Vector2d(-18, 55))

                //3rd CONE
                .lineTo(new Vector2d(-18, 30))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    //move arm
                })
                .lineTo(new Vector2d(-18, 55))

                .build();
        mecanumDrive.followTrajectorySequence(sequence);
    }
}

