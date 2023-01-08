package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.ScissorLift;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicInteger;

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
        mecanumDrive = drive.getInstance();
        lift.resetEncoder();
        startPose = new Pose2d(-72, 36, 0);
        conePosition = VisionPipeline.ConePosition.NULL;
        telemetry.setAutoClear(true);
        vision.initVision();
        while (opModeInInit()) {
            conePosition = VisionPipeline.ConePosition.POS1;
            telemetry.addData("Cone Position", conePosition);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        AtomicInteger custom = new AtomicInteger(1000);
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
                .splineTo(new Vector2d(-18, 60), Math.toRadians(90))
                //1+1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.HIGH);
                })
                .back(35)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.CUSTOM.setCustom(custom.get()));
                })
                .forward(35)
                //1+2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.HIGH);
                })
                .back(35)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.CUSTOM.setCustom(custom.addAndGet(-50)));
                })
                .forward(35)
                //1+3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.HIGH);
                })
                .back(35)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.CUSTOM.setCustom(custom.addAndGet(-50)));
                })
                .forward(35)
                //1+4
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.HIGH);
                })
                .back(35)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.CUSTOM.setCustom(custom.addAndGet(-50)));
                })
                .forward(35)
                //1+5
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.HIGH);
                })
                .back(35)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.moveAndDrop(ScissorLift.Level.RESET);
                })
                .forward(park)
                .build();
        mecanumDrive.followTrajectorySequence(sequence);
    }
}

