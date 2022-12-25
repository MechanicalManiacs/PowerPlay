package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class RedLeftAuto extends FishloAutonomousProgram {

    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    VisionPipeline.ConePosition conePosition;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-72, 36, 0);
        conePosition = VisionPipeline.ConePosition.NULL;
        telemetry.setAutoClear(true);
        while (opModeInInit()) {
            conePosition = vision.getConePosition();
            telemetry.addData("Cone Position", conePosition);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        int park = 0;
        switch (conePosition) {
            case POS1:
                park = 35;
                break;
            case POS2:
                park = 20;
                break;
            case POS3:
                park = -20;
        }
        mecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence sequence = mecanumDrive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-30,36))
                .turn(Math.toRadians(-30))
                .forward(5)
                .back(5)
                .turn(Math.toRadians(30))
                .splineTo(new Vector2d(-18, 60), Math.toRadians(90))
                //1st CONE
                .back(35)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    //move arm
                })
                .forward(35)

                //2nd CONE
                .back(35)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    //move arm
                })
                .forward(35)

                //3rd CONE
                .back(35)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    //move arm
                })
                .forward(park)
                .build();
        mecanumDrive.followTrajectorySequence(sequence);
    }
}

