package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;


@Config
public class VisionTest extends FishloAutonomousProgram {

    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    VisionPipeline.ConePosition pos;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = drive.getInstance();
        telemetry.addLine("Initialized");
        telemetry.update();
        telemetry.setAutoClear(true);
        telemetry.addLine("INIT VISION");
        telemetry.update();
        vision.initVision();
        startPose = new Pose2d(72, -36, Math.toRadians(0));
        mecanumDrive.setPoseEstimate(startPose);
        while (!isStarted()) {
            pos = vision.getConePosition();
            telemetry.addData("Cone Position", pos);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        telemetry.addLine("ENTERED MAIN");
        telemetry.update();
        Trajectory traj = null;
        telemetry.addLine("RUNNING");
        telemetry.update();
        switch (pos) {
            case POS1:
                telemetry.addLine("SETTING");
                telemetry.update();
                traj = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(36,-72),Math.toRadians(0))
                        .build();
                break;
            case POS2:
                telemetry.addLine("SETTING");
                telemetry.update();
                traj = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                        .build();
                break;
            case POS3:
                telemetry.addLine("SETTING");
                telemetry.update();
                traj = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(36, 10), Math.toRadians(0))
                        .build();
            default:
                telemetry.addLine("SETTING");
                telemetry.update();
                traj = mecanumDrive.trajectoryBuilder(startPose)
                        .forward(36)
                        .build();
        }
        telemetry.addLine("FOLLOWING");
        telemetry.update();
        mecanumDrive.followTrajectory(traj);
    }


}
