package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.jetbrains.annotations.NotNull;
import java.util.Vector;

public class PreloadParkingBlueLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(11.0, 11.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 64, Math.toRadians(270)))
                                // Preload
                                .splineTo(new Vector2d(40,52), Math.toRadians(330))
                                // Preplaced
                                .lineToLinearHeading(new Pose2d(36, 52, Math.toRadians(270)))
                                // park 2
                                .lineToSplineHeading(new Pose2d(36, 34, Math.toRadians(270)))
                                // park 3
                                .lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(270)))
                                // park 1
                                //.lineToLinearHeading(new Pose2d(60, 34, Math.toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


