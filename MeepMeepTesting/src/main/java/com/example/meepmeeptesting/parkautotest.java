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

public class parkautotest {
    public static void main(String[] args) {




        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.2)
                .setDimensions(12.5984, 12.5984)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60,-36, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(33,-36), Math.toRadians(180))
                                .lineTo(new Vector2d(33,-12))
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