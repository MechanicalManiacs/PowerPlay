package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

public class LEFTAUTO extends FishloAutonomousProgram {

    VisionPipeline.ConePosition position = VisionPipeline.ConePosition.NULL;
    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = drive.getInstance();
        vision.initVision();
        startPose = new Pose2d();
    }

    @Override
    public void main() {
//        TrajectorySequence theWholeFuckingAuto = mecanumDrive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(-24, 0), Math.toRadians(45), )
    }
}