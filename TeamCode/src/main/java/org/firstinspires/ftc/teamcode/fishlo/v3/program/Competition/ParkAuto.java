package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class ParkAuto extends FishloAutonomousProgram {

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
        startPose = new Pose2d(60, -36, Math.toRadians(180));
        telemetry.addLine("Initialized");
        telemetry.update();
        telemetry.setAutoClear(true);
        vision.initVision();
        while (!isStarted()) {
            conePosition = vision.getConePosition();
            telemetry.addData("Cone Position", conePosition);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        if (conePosition == VisionPipeline.ConePosition.NULL) conePosition = VisionPipeline.ConePosition.POS3;
        mecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence traj = mecanumDrive.trajectorySequenceBuilder(startPose).forward(5).build();
        switch (conePosition) {
            case POS1:
                traj = mecanumDrive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(33,-36), Math.toRadians(180))
                        .lineTo(new Vector2d(33,-60))
                        .build();
                break;
            case POS2:
                traj = mecanumDrive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(40,-36), Math.toRadians(180))
                        .build();
                break;
            case POS3:
                traj = mecanumDrive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(33,-36), Math.toRadians(180))
                        .lineTo(new Vector2d(33,-12))
                        .build();
        }

        mecanumDrive.followTrajectorySequence(traj);
    }
}

