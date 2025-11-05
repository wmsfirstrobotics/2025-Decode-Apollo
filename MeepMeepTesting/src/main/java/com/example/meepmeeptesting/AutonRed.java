package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutonRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 49, Math.toRadians(-54)))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-11.5, 27, Math.toRadians(-270)), Math.toRadians(-270))
                .lineToY(47.5)
                .splineToLinearHeading(new Pose2d(-23.5, 23.5, Math.toRadians(-54)), Math.toRadians(200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(11.5, 27, Math.toRadians(-270)), Math.toRadians(-270))
                .lineToY(47.5)
                .splineToLinearHeading(new Pose2d(-23.5, 23.5, Math.toRadians(-54)), Math.toRadians(200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(35.5, 27, Math.toRadians(-270)), Math.toRadians(-270))
                .lineToY(47.5)
                .splineToLinearHeading(new Pose2d(-23.5, 23.5, Math.toRadians(-54)), Math.toRadians(200))
                .waitSeconds(2)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
