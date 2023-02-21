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
public class LEFTAUTOV2 extends FishloAutonomousProgram {

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
        System.out.println(5);
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
        lift.setClaw(LinearSlide.ClawPos.CLOSED);
        mecanumDrive.setPoseEstimate(startPose);

        //1+0
        mecanumDrive.followTrajectorySequence(traj1);
        drive.strafeUntilAligned("RIGHT");
        mecanumDrive.update();

        traj2 = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
                .forward(drive.getDistance() > 5.5 ? drive.getDistance() - 5.5 : 5.5 - drive.getDistance())
                .strafeRight(2)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.CYCLE_POS_1, 1);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0, () -> {
                    lift.setClaw(LinearSlide.ClawPos.OPEN);
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(85))
                .build();

//        //1+1
//        mecanumDrive.followTrajectorySequence(traj2);
//        drive.strafeUntilAligned("RIGHT");
//        mecanumDrive.update();
//
//        traj2 = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
//                .forward(drive.getDistance() > 6.5 ? drive.getDistance() - 6.5 : 6.5 - drive.getDistance())
//                .strafeRight(2)
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.liftSlide(LinearSlide.Level.CYCLE_POS_2, 1);
//                })
//                .waitSeconds(0.5)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.setClaw(LinearSlide.ClawPos.OPEN);
//                })
//                .back(2)
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                .strafeRight(12)
//                .resetVelConstraint()
//                .waitSeconds(0.5)
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                .forward(48)
//                .resetVelConstraint()
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.setClaw(LinearSlide.ClawPos.CLOSED);
//                    cycleTimer.reset();
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.liftSlide(LinearSlide.Level.HIGH, 1);
//                })
//                .back(46)
//                .build();
//
//        //1+2
//        mecanumDrive.followTrajectorySequence(traj2);
//        drive.strafeUntilAligned("RIGHT");
//        mecanumDrive.update();
//
//        traj2 = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
//                .forward(drive.getDistance() > 6.5 ? drive.getDistance() - 6.5 : 6.5 - drive.getDistance())
//                .strafeRight(3)
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.liftSlide(LinearSlide.Level.CYCLE_POS_3,1);
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.setClaw(LinearSlide.ClawPos.OPEN);
//                })
//                .back(2)
//                .strafeRight(6)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.liftSlide(LinearSlide.Level.RESET, 1);
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.setClaw(LinearSlide.ClawPos.OPEN);
//                })
//                .back(3)
//                .build();
//        mecanumDrive.followTrajectorySequence(traj2);
//        mecanumDrive.update();

//        //1+3
//        mecanumDrive.followTrajectorySequence(traj2);
//        drive.strafeUntilAligned("RIGHT");
//        mecanumDrive.update();
//
//        traj2 = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.liftSlide(LinearSlide.Level.CYCLE_POS_4,1);
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    lift.setClaw(LinearSlide.ClawPos.OPEN);
//                })
//                .back(2)
//                .strafeRight(10)
//                .build();
//        mecanumDrive.followTrajectorySequence(traj2);

        if (position == VisionPipeline.ConePosition.POS1){
            mecanumDrive.followTrajectorySequence(park1);
        }
        else if (position == VisionPipeline.ConePosition.POS2){
            mecanumDrive.followTrajectorySequence(park2);
        }
        else {
            mecanumDrive.followTrajectorySequence(park3);
        }
    }

    public void generateTrajectory() {
        traj1 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.liftSlide(LinearSlide.Level.HIGH, 1);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .forward(60)
                .back(10)
                .build();

        park1 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .strafeLeft(2)
                .build();
        park2 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .strafeRight(15)
                .build();
        park3 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .strafeRight(48)
                .build();

    }
}
