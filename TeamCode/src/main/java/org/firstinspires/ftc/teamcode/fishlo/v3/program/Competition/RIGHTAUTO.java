package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.LinearSlide;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class RIGHTAUTO extends FishloAutonomousProgram {

    VisionPipeline.ConePosition position = VisionPipeline.ConePosition.NULL;
    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.setAutoClear(true);
        mecanumDrive = drive.getInstance();
        telemetry.addLine("ROADRUNNER INITIALIZED");
        telemetry.update();
        vision.initVision();
        telemetry.addLine("VISION INITIALIZED");
        telemetry.update();
        startPose = new Pose2d(36, -60, Math.toRadians(90));
        telemetry.addLine("STARTING VISION DETECTION");
        while (!isStarted()) {
            position = vision.getConePosition();
            telemetry.addData("CONE POSITION", position);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        lift.setClaw(LinearSlide.ClawPos.CLOSED);
        mecanumDrive.setPoseEstimate(startPose);
        double park = 0;
        switch (position) {
            case POS1:
                park = -60;
                break;
            case POS2:
                park = -36;
                break;
            case POS3:
                park = -12;
                break;
            default:
                park = -36;
                break;
        }
        double finalPark = park;
        TrajectorySequence traj = mecanumDrive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(40, -24), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    lift.liftSlide(LinearSlide.Level.HIGH);
                })
                .splineTo(new Vector2d(40, -4), Math.toRadians(125))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.CYCLE_POS_1);
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    lift.setClaw(LinearSlide.ClawPos.CLOSED);
                })
                .waitSeconds(0.5)
                .splineTo(new Vector2d(finalPark, -12), Math.toRadians(90))
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(60,-12), Math.toRadians(0))
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    lift.liftSlide(LinearSlide.Level.LOW);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.deposit(LinearSlide.CyclePos.TWO);
//                })
//                .waitSeconds(0.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)), Math.toRadians(-180))
//                .setReversed(false)
//                .addDisplacementMarker(() -> {
//                    lift.setClaw(LinearSlide.ClawPos.CLOSED);
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    lift.liftSlide(LinearSlide.Level.HIGH);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.deposit(LinearSlide.CyclePos.THREE);
//                })
//                .waitSeconds(0.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)), Math.toRadians(-180))
//                .setReversed(false)
//                .addDisplacementMarker(() -> {
//                    lift.setClaw(LinearSlide.ClawPos.CLOSED);
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    lift.liftSlide(LinearSlide.Level.HIGH);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.deposit(LinearSlide.CyclePos.FOUR);
//                })
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(finalPark, 12, Math.toRadians(-90)), Math.toRadians(-90))
//                .setReversed(false)
                .build();
        mecanumDrive.followTrajectorySequence(traj);
    }
}
