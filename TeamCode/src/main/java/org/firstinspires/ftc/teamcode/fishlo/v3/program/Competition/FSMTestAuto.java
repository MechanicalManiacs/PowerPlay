package org.firstinspires.ftc.teamcode.fishlo.v3.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fishlo.v3.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.LinearSlide;
import org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class FSMTestAuto extends FishloAutonomousProgram {

    SampleMecanumDrive mecanumDrive;
    VisionPipeline.ConePosition position = VisionPipeline.ConePosition.NULL;

    private enum State {
        DETECT,
        TRAJECTORY_1,
        TRAJECTORY_SCORE,
        REPEAT_1,
        REPEAT_A,
        REPEAT_2,
        REPEAT_B,
        PARK,
        IDLE
    }

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();

    private static double correction = 2;

    State state = State.IDLE;
    LinearSlide.Level currCyclePos = LinearSlide.Level.CYCLE_POS_5;

    Pose2d startPose = new Pose2d(-35, 61.5, Math.toRadians(270));
    Pose2d rightPrimaryPose = new Pose2d(-20, 12, Math.toRadians(270));
    Pose2d rightStackPose = new Pose2d(-55,10.5, Math.toRadians(180));
    Pose2d parkPose1 = new Pose2d(-10,14.5, Math.toRadians(270));
    Pose2d parkPose2 = new Pose2d(-35, 12, Math.toRadians(270));
    Pose2d parkPose3 = new Pose2d(-58, 14.5, Math.toRadians(270));

    TrajectorySequence trajectory1;
    TrajectorySequence trajectoryScore;
    TrajectorySequence repeat1;
    TrajectorySequence repeat2;
    TrajectorySequence park1;
    TrajectorySequence park2;
    TrajectorySequence park3;
    TrajectorySequence parkToRun;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.setAutoClear(true);
        telemetry.addLine("INITIALIZING ROADRUNNER");
        telemetry.update();
        mecanumDrive = drive.getInstance();
        sleep(100);
        telemetry.addLine("INITIALIZING VISION");
        telemetry.update();
        vision.initVision();
        sleep(100);
        startPose = new Pose2d(36, -60, Math.toRadians(90));
        telemetry.addLine("BUILDING TRAJECTORY SEQUENCES");
        telemetry.update();
        mecanumDrive.setPoseEstimate(startPose);
        trajectory1 = mecanumDrive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(rightPrimaryPose, 0)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    lift.liftSlide(LinearSlide.Level.HIGH);
                })
                .forward(correction)
                .build();
        trajectoryScore = mecanumDrive.trajectorySequenceBuilder(trajectory1.end())
                .back(correction)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.liftSlide(currCyclePos);
                })
                .build();
        repeat1 = mecanumDrive.trajectorySequenceBuilder(trajectoryScore.end())
                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    lift.liftSlide(currCyclePos.next());
                })
                .build();
        repeat2 = mecanumDrive.trajectorySequenceBuilder(repeat1.end())
                .back(12)
                .lineToSplineHeading(rightPrimaryPose)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    lift.liftSlide(LinearSlide.Level.HIGH);
                })
                .forward(correction)
                .build();
        park1 = mecanumDrive.trajectorySequenceBuilder(repeat2.end())
                .lineToSplineHeading(parkPose1)
                .back(24)
                .build();
        park2 = mecanumDrive.trajectorySequenceBuilder(repeat2.end())
                .lineToSplineHeading(parkPose2)
                .back(24)
                .build();
        park3 = mecanumDrive.trajectorySequenceBuilder(repeat2.end())
                .lineToSplineHeading(parkPose3)
                .back(24)
                .build();
        sleep(100);
        telemetry.addLine("DETECTION STARTED");
        telemetry.update();
        sleep(100);
        while (!isStarted()) {
            position = vision.getConePosition();
            telemetry.addData("CONE POSITION", position);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        if (isStopRequested()) return;

        autoTimer.reset();
        state = State.DETECT;

        while (opModeIsActive()) {
            switch (state) {
                case DETECT:
                    state = State.TRAJECTORY_1;
                    mecanumDrive.followTrajectorySequenceAsync(trajectory1);
                    break;
                case TRAJECTORY_1:
                    if (!mecanumDrive.isBusy()) {
                        state = State.TRAJECTORY_SCORE;
                    }
                    break;
                case TRAJECTORY_SCORE:
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 0.5) {
                        lift.setClaw(LinearSlide.ClawPos.OPEN);
                    }
                    mecanumDrive.followTrajectorySequenceAsync(trajectoryScore);
                    state = State.REPEAT_A;
                    break;
                case REPEAT_A:
                    if (!mecanumDrive.isBusy()) {
                        state = State.REPEAT_1;
                    }
                    break;
                case REPEAT_1:
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        lift.liftSlide(currCyclePos);
                    }
                    if (currCyclePos == LinearSlide.Level.END) {
                        state = State.PARK;
                    }
                    else if (autoTimer.seconds() < 22) {
                        state = State.REPEAT_B;
                        mecanumDrive.followTrajectorySequenceAsync(repeat1);
                    }
                    else {
                        state = State.PARK;
                    }
                    break;
                case REPEAT_B:
                    if (!mecanumDrive.isBusy()) {
                        state = State.REPEAT_2;
                    }
                    break;
                case REPEAT_2:
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        lift.liftSlide(currCyclePos.next());
                    }
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 0.5) {
                        lift.setClaw(LinearSlide.ClawPos.CLOSED);
                    }
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        lift.liftSlide(currCyclePos);
                    }
                    currCyclePos = currCyclePos.next().next();
                    mecanumDrive.followTrajectorySequenceAsync(repeat2);
                    state = State.TRAJECTORY_1;
                    break;
                case PARK:
                    switch (position) {
                        case POS1:
                            parkToRun = park1;
                            break;
                        case POS2:
                            parkToRun = park2;
                            break;
                        case POS3:
                            parkToRun = park3;
                            break;
                    }
                    state = State.IDLE;
                    mecanumDrive.followTrajectorySequenceAsync(parkToRun);
                    break;
            }
            mecanumDrive.update();
            lift.update();
        }
    }
}
