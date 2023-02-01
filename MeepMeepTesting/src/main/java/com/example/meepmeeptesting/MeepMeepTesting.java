package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Timer;
import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {

        int pos = 2;
        double park = 0;    
        switch (pos) {
            case 1:
                park = -60;
                break;
            case 2:
                park = -36;
                break;
            case 3:
                park = -12;
                break;
            default:
                park = -36;
                break;
        }

        MeepMeep meepMeep = new MeepMeep(800);

        double finalPark = park;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.2)
                .setDimensions(12.5984, 12.5984)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                .splineTo(new Vector2d(-36, 18), Math.toRadians(-90))
                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                                    liftSlide();
                                })
                                .splineTo(new Vector2d(-29, 5.5), Math.toRadians(-45))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    deposit();
                                })
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    clawClose();
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                    liftSlide();
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    deposit();
                                })
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    clawClose();
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                    liftSlide();
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    deposit();
                                })
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    clawClose();
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                    liftSlide();
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    deposit();
                                })
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    clawClose();
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                    liftSlide();
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    deposit();
                                })
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)), Math.toRadians(-180))
                                .setReversed(false)
                                .addDisplacementMarker(() -> {
                                    clawClose();
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                    liftSlide();
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-29, 5.5, Math.toRadians(-45)), Math.toRadians(-45))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    deposit();
                                })
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(finalPark, 12, Math.toRadians(-90)), Math.toRadians(-90))
                                .setReversed(false)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void liftSlide() {

    }

    static int cycle = 0;
    static int cones = 0;
    public static void deposit() {
        System.out.println("1+" + cones);
        cones++;
        retractSlideToCycle(cycle);
        openClaw();
        cycle++;
    }

    public static void openClaw() {

    }

    public static void clawClose() {

    }

    public static void retractSlideToCycle(int cycle) {

    }
}