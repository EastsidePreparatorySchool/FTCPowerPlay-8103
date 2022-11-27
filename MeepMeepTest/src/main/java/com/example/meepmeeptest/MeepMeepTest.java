package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String args[]) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 55, 5.27, Math.toRadians(360), 12.5)
                .setDimensions(12.25,17.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, 62.125, Math.toRadians(90)))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(32, 8, Math.toRadians(45)), Math.toRadians(-100))
                                /*.addTemporalMarker(2, () -> {
                                    // run slides here to place cone
                                })*/
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(62, 10.5, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(32, 8, Math.toRadians(45)), Math.toRadians(-140))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}