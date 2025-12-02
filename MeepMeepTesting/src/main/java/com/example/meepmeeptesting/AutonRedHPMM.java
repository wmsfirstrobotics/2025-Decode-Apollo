//FINISHED (as of 12/1/25)


package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutonRedHPMM {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(100, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62.5, 18, Math.toRadians(180)))

                .splineToSplineHeading(new Pose2d(-23.5, 23.5, Math.toRadians(-54)), Math.toRadians(200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-11.5, 27, Math.toRadians(-270)), Math.toRadians(-270))
                .strafeTo(new Vector2d(-11.5, 47.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, 23.5, Math.toRadians(-54)), Math.toRadians(200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(11.5, 27, Math.toRadians(-270)), Math.toRadians(-270))
                .strafeTo(new Vector2d(11.5, 47.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, 23.5, Math.toRadians(-54)), Math.toRadians(200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(35.5, 27, Math.toRadians(-270)), Math.toRadians(-270))
                .strafeTo(new Vector2d(35.5, 47.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
