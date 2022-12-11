package org.firstinspires.ftc.teamcode.fishlo.v3.program.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class redLowAutonomous extends FishloAutonomousProgram {
    SampleMecanumDrive mdrive;
    Pose2d startPose;
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {

    }

    @Override
    public void main() {
        mdrive = new SampleMecanumDrive(hardwareMap);
        mdrive.setPoseEstimate(new Pose2d());
        startPose = new Pose2d(60, -36, Math.toRadians(90));
        // goes to (36, -60)
        mdrive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(45))
                .forward(23.5)
                .turn(Math.toRadians(-45))
                .forward(24)
                .addDisplacementMarker(() -> {
                  //drop cone on low junction
                })
                .build();

    }


}