package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 63, Math.toRadians(270)))
                                .splineTo(new Vector2d(-42, 36), Math.toRadians(240))

                               .addDisplacementMarker(() -> {
                                    // This marker runs after the first splineTo()
                                    //drop
                                    // Run your action in here!
                                })
                                .splineTo(new Vector2d(-52, 35), Math.toRadians(180))
                                .addDisplacementMarker(() -> {
                                    // This marker runs after the first splineTo()
                                    //Pick up
                                    // Run your action in here!
                                })
                                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                                .splineTo(new Vector2d(43, 38), Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    // This marker runs after the first splineTo()
                                    //Backb
                                    // Run your action in here!
                                })
                                .splineTo(new Vector2d(-50, 35), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
