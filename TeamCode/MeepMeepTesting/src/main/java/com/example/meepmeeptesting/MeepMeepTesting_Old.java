package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_Old {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11, 11)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)

                .followTrajectorySequence(park ->
                        park.trajectorySequenceBuilder(new Pose2d(37, 57, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38, 53, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(10, 57, Math.toRadians(165)))
                                .lineToSplineHeading(new Pose2d(18, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(60, 12, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(18, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(60, 12, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(18, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(60, 12, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(18, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(60, 12, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(18, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(8, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(60, 12, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(18, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(58, 12, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(58, 35, Math.toRadians(0)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}