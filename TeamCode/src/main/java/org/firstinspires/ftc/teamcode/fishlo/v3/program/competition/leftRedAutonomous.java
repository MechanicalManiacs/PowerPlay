package org.firstinspires.ftc.teamcode.fishlo.v3.program.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class leftRedAutonomous extends FishloAutonomousProgram {
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
       // TrajectorySequence ts = mdrive.trajectorySequenceBuilder();///////////////////////////////////////need to fix
        startPose = new Pose2d(60, -36, Math.toRadians(90));
        // goes to (36, -60)
        if (vision.getConePosition() == VisionPipeline.ConePosition.POS1) {
            mdrive.trajectorySequenceBuilder(startPose)
                    .forward(24)
                    .turn(Math.toRadians(45))
                    .addDisplacementMarker(() -> {
                        //drop cup on to low junction
                    })
                    .turn(Math.toRadians(45))
                    .forward(24)
                    .build();
        }
        if (vision.getConePosition() == VisionPipeline.ConePosition.POS2) {
            mdrive.trajectorySequenceBuilder(startPose)
                    .forward(24)
                    .turn(Math.toRadians(45))
                    .addDisplacementMarker(() -> {
                        //drop cup on to low junction
                    })
                    .build();
        }
        if (vision.getConePosition() == VisionPipeline.ConePosition.POS3) {
            mdrive.trajectorySequenceBuilder(startPose)
                    .forward(24)
                    .turn(Math.toRadians(45))
                    .addDisplacementMarker(() -> {
                        //drop cup on to low junction
                    })
                    .turn(Math.toRadians(-135))
                    .forward(24)
                    .build();
        }
    }


}
