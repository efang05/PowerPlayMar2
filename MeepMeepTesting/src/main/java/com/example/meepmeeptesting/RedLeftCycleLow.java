package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeftCycleLow {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 30, Math.toRadians(180), Math.toRadians(180), 9.5)
                .setDimensions(11.0, 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(180)))
                                // Preload
                                .lineTo(new Vector2d(-36,-12))
                                // Cycle #1
                                .setReversed(true)
                                .lineTo(new Vector2d(-52, -12))
                                .lineTo(new Vector2d(-49, -12))
                                .lineTo(new Vector2d(-52, -12))
                                .lineTo(new Vector2d(-49, -12))
                                .lineTo(new Vector2d(-52, -12))
                                .lineTo(new Vector2d(-49, -12))
                                // Park
                                .setReversed(true)
                                .lineTo(new Vector2d(-36,-12))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}