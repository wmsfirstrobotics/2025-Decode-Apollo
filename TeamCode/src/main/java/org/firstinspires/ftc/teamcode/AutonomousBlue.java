package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;

public class AutonomousBlue {
    public static void main(String[] args) {
        MecanumDrive myBot = new MecanumDrive();


        myBot.runAction(myBot.actionBuilder(new Pose2d(-61, -33, Math.toRadians(0)))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-11.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-47.5)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(11.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-47.5)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(35.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-47.5)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(2)
                .build());

    }
}
