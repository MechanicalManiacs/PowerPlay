package org.firstinspires.ftc.teamcode.fishlo.v3.program.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.dfp.DfpField;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Autonomous
@Config
public class VisionTest extends FishloAutonomousProgram {

    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    VisionPipeline.ConePosition pos;

    double sum;
    double count;

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
        vision.initVision();
//        sum = 0;
//        count = 0;
        while (!isStarted()) {
            pos = VisionPipeline.ConePosition.POS1;
            telemetry.addData("Cone Position", pos);
            telemetry.update();
            System.out.println("Init pos: " + pos);
//            String p = pos.toString();
//            int len = p.length();
//            int num = Integer.parseInt(Character.toString(p.charAt(len-1)));
//            sum += num;
//            count++;
        }
    }

    @Override
    public void main() {
        mecanumDrive.setPoseEstimate(startPose);
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
