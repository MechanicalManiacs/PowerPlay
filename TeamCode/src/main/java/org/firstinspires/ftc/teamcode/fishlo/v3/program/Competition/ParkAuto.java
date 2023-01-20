package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

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
        startPose = new Pose2d(-72, 36, 0);
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
        mecanumDrive.setPoseEstimate(startPose);
        switch (conePosition) {
            case POS1:
                Trajectory traj = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(36,-72),Math.toRadians(0))
                                .build();
                mecanumDrive.followTrajectory(traj);
                break;
            case POS2:
                Trajectory traj2 = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                        .build();
                mecanumDrive.followTrajectory(traj2);
                break;
            case POS3:
                Trajectory traj3 = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(36, 10), Math.toRadians(0))
                        .build();
                mecanumDrive.followTrajectory(traj3);
        }





    }
}

