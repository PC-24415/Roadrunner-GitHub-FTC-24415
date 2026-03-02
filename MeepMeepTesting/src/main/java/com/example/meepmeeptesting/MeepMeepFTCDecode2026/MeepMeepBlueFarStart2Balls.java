package com.example.meepmeeptesting.MeepMeepFTCDecode2026;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueFarStart2Balls {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -14, Math.toRadians(180)))
                .splineToLinearHeading( new Pose2d(50,-12,Math.toRadians(205)), Math.toRadians(205))
                .waitSeconds(3.5)
                .splineTo(new Vector2d(30,-30), 3 * Math.PI / 2)
                .lineToY(-60)
                .lineToY(-55)
                .splineToLinearHeading( new Pose2d(50,-12,Math.toRadians(205)), Math.toRadians(205))
                .waitSeconds(3.5)
                .splineToLinearHeading(new Pose2d(8, -35, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-55)
                .lineToY(-35)
                .splineTo(new Vector2d(50,-12), Math.toRadians(205- 180))
                .waitSeconds(3.5)
                .build());

        myBot.setDimensions(18.0,18.0);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}