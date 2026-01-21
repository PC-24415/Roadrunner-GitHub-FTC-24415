package com.example.meepmeeptesting.MeepMeepFTCDecode2026;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueCloseStartTwoBalls {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -48, Math.toRadians(225)))
                .lineToY(-14)
                //Shoot
                .waitSeconds(5)
                //Intake ON
                .splineTo(new Vector2d(-12,-49), Math.toRadians(270))
                //Intake Off
                .splineToLinearHeading(new Pose2d(-24,-24,Math.toRadians(135 + 90)), Math.toRadians(135 + 90))
                //Shoot
                .waitSeconds(5)
                //Intake On
                .lineToY(-12)
                .splineToLinearHeading(new Pose2d(12,-36, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-50)
                //Intake Off
                .lineToY(-30)
                .splineToLinearHeading(new Pose2d(-24,-24,Math.toRadians(135 + 90)), Math.toRadians(135 + 90))
                //Shoot
                .waitSeconds(5)
                .build());

        myBot.setDimensions(18.0,18.0);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}