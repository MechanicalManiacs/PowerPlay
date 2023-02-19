package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.LinearSlide;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class RIGHTAUTO extends FishloAutonomousProgram {

    VisionPipeline.ConePosition position = VisionPipeline.ConePosition.NULL;
    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence park1;
    TrajectorySequence park2;
    TrajectorySequence park3;

    ElapsedTime timer;
    ElapsedTime cycleTimer;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        lift.resetEncoder();
        startPose = new Pose2d(36, -67, Math.toRadians(90));
        timer = new ElapsedTime();
        cycleTimer = new ElapsedTime();
        telemetry.addLine("Initialized");
        telemetry.update();
        vision.initVision();

        telemetry.addLine("Building Trajectory Sequence...");
        telemetry.update();
        generateTrajectory();
        telemetry.addLine("DONE");
        telemetry.update();

        telemetry.clear();
        telemetry.setAutoClear(true);

        while (!isStarted()) {
            position = vision.getConePosition();
            telemetry.addData("Cone Position", position);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        timer.reset();
        AtomicReference<Double> time = new AtomicReference<>((double) 0);
        lift.setClaw(LinearSlide.ClawPos.CLOSED);
        mecanumDrive.setPoseEstimate(startPose);

        //1+0
        mecanumDrive.followTrajectorySequence(traj1);
        drive.strafeUntilAligned("LEFT");
        mecanumDrive.update();

        traj2 = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
                .forward(drive.getDistance() > 5.5 ? drive.getDistance() - 5.5 : 5.5 - drive.getDistance())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.CYCLE_POS_1, 1);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0, () -> {
                    lift.setClaw(LinearSlide.ClawPos.OPEN);
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(-85))
                .forward(35)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.setClaw(LinearSlide.ClawPos.CLOSED);
                    cycleTimer.reset();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.HIGH, 1);
                })
                .back(4)
                .turn(Math.toRadians(90))
                .build();

        //1+1
        mecanumDrive.followTrajectorySequence(traj2);
        drive.strafeUntilAligned("LEFT");
        mecanumDrive.update();

        traj2 = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
                .forward(drive.getDistance() > 5.5 ? drive.getDistance() - 5.5 : 5.5 - drive.getDistance())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.CYCLE_POS_2, 1);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.setClaw(LinearSlide.ClawPos.OPEN);
                })
                .back(2)
                .turn(Math.toRadians(-84))
                .forward(35)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.setClaw(LinearSlide.ClawPos.CLOSED);
                    time.set(cycleTimer.seconds());
                    cycleTimer.reset();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.HIGH, 1);
                })
                .back(4)
                .turn(Math.toRadians(84))
                .strafeLeft(15)
                .build();

        //1+2
        mecanumDrive.followTrajectorySequence(traj2);
        drive.strafeUntilAligned("LEFT");
        mecanumDrive.update();

        traj2 = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
                .forward(drive.getDistance() > 5.5 ? drive.getDistance() - 5.5 : 5.5 - drive.getDistance())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.RESET,1);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.setClaw(LinearSlide.ClawPos.OPEN);
                })
                .back(2)
                .build();

        if (position == VisionPipeline.ConePosition.POS1){
            mecanumDrive.followTrajectorySequence(park1);
        }
        else if (position == VisionPipeline.ConePosition.POS2){
            mecanumDrive.followTrajectorySequence(park2);
        }
        else {
            mecanumDrive.followTrajectorySequence(park3);
        }
        PoseStorage.currentPose = mecanumDrive.getPoseEstimate();
        telemetry.addLine("TIME")
                .addData("TOTAL TIME", timer.seconds())
                .addData("AVG CYCLE TIME", time.get());
        telemetry.update();
    }

    public void generateTrajectory() {
        traj1 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.HIGH, 1);
                })
                .forward(48)
                .strafeLeft(5)
                .build();

        park1 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .strafeLeft(12)
                .build();
        park2 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .strafeRight(12)
                .build();
        park3 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .strafeRight(30)
                .build();

    }
}
