package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.jetbrains.annotations.NotNull;
import java.util.Vector;

public class PreloadTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(11.0, 11.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -64, Math.toRadians(90)))
                                // Preload
                                .splineTo(new Vector2d(40,-52), Math.toRadians(30))
                                // Preplaced
                                .lineToLinearHeading(new Pose2d(36, -52, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(40, -12, Math.toRadians(0)))
                                // Cycle #1
                                .lineTo(new Vector2d(61, -12))
                                .setReversed(true)
                                .splineTo(new Vector2d(24, -8), Math.toRadians(90))
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
//                                // Cycle #2
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
//                                .lineTo(new Vector2d(61, -12))
//                                .setReversed(false)
//                                .lineTo(new Vector2d(40, -12))
//                                .splineToConstantHeading(new Vector2d(24, -8), Math.toRadians(90))
//                                // Cycle #3
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
//                                .lineTo(new Vector2d(61, -12))
//                                .setReversed(false)
//                                .lineTo(new Vector2d(40, -12))
//                                .splineToConstantHeading(new Vector2d(24, -8), Math.toRadians(90))
//                                // Cycle #4
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(0))
//                                .lineTo(new Vector2d(61, -12))
//                                .setReversed(false)
//                                .lineTo(new Vector2d(40, -12))
//                                .splineToConstantHeading(new Vector2d(24, -8), Math.toRadians(90))
//                                // Park
//                                .setReversed(true)
//                                .lineToLinearHeading(new Pose2d(36, -24, Math.toRadians(90)))
//                                .splineToLinearHeading(new Pose2d(62, -36), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}