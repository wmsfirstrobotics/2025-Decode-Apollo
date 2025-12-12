package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "AutonBlue TV Side", group = "Autonomous", preselectTeleOp = ("LiLiLocalizationTest"))

public class AutonBlueTVSide extends LinearOpMode {

    private DcMotorEx indexer;
    private DcMotor intake;
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(-62.5, -39.5, Math.toRadians(0)));
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter.setVelocityPIDFCoefficients(60, 0, 0, 13.2);
        rightShooter.setVelocityPIDFCoefficients(60, 0, 0, 13.2);

        Action trajectory1 = myBot.actionBuilder(new Pose2d(-62.5, -39.5, Math.toRadians(0)))

                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(47)), Math.toRadians(35))
                .build();
        //shoot after this

        Action trajectory2 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(47)))

                .splineToLinearHeading(new Pose2d(-10.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .build();
        // start intaking motor

        Action trajectory3 = myBot.actionBuilder(new Pose2d(-10.5, -27, Math.toRadians(270)))

                .strafeTo(new Vector2d(-10.5, -50.5), new TranslationalVelConstraint(13))
                .build();
        // stop intaking motor

        Action trajectory4 = myBot.actionBuilder(new Pose2d(-10.5, -47.5, Math.toRadians(270)))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(48)), Math.toRadians(-200))
                .build();
        // start shooting stuff

        Action trajectory5 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(48)))

                .splineToLinearHeading(new Pose2d(14.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .build();
//start intaking motor

        Action trajectory6 = myBot.actionBuilder(new Pose2d(14.5, -27, Math.toRadians(270)))

                .strafeTo(new Vector2d(14.5, -50.5), new TranslationalVelConstraint(13))
                .build();
        //stop intaking motor

        Action trajectory7 = myBot.actionBuilder(new Pose2d(14.5, -50.5, Math.toRadians(270)))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(47)), Math.toRadians(-200))
                .build();
        //start shooting stuff

        Action trajectory8 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(47)))

                .splineToLinearHeading(new Pose2d(39.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .build();
        //start intaking motor

        Action trajectory9 = myBot.actionBuilder(new Pose2d(39.5, -27, Math.toRadians(270)))
                .strafeTo(new Vector2d(39.5, -50.5), new TranslationalVelConstraint(13))
                .build();
        //stop intaking motor

        Action trajectory10 = myBot.actionBuilder(new Pose2d(30.5, -50.5, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(90))
                .build();
        //stop at in front of gate, 3 artifacts loaded for teleop. at this point there should be 9 artifacts in the classifier.

        waitForStart();

        leftShooter.setVelocity(935);
        rightShooter.setVelocity(-935);
        Actions.runBlocking(trajectory1);
        //shoot 3 balls
        indexer.setPower(-1);
        intake.setPower(-0.5);
        sleep(900);
        intake.setPower(-1);
        sleep(700);
        indexer.setPower(0);
        intake.setPower(0);



        Actions.runBlocking(trajectory2);
        // start intaking motor
        intake.setPower(-1);
        indexer.setPower(0.25);
        Actions.runBlocking(trajectory3);
        // stop intaking motor
        intake.setPower(0);
        indexer.setPower(0);
        Actions.runBlocking(trajectory4);
        //shoot 3 balls
        indexer.setPower(-1);
        intake.setPower(-0.5);
        sleep(900);
        intake.setPower(-1);
        sleep(700);
        indexer.setPower(0);
        intake.setPower(0);

        Actions.runBlocking(trajectory5);
        //start intaking motor
        intake.setPower(-1);
        indexer.setPower(0.25);
        Actions.runBlocking(trajectory6);
        //stop intaking motor
        intake.setPower(0);
        indexer.setPower(0);
        Actions.runBlocking(trajectory7);
        // shooting
        indexer.setPower(-1);
        intake.setPower(-0.5);
        sleep(900);
        intake.setPower(-1);
        sleep(700);
        indexer.setPower(0);
        intake.setPower(0);

        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
        Actions.runBlocking(trajectory8);
        //start intaking motor
        intake.setPower(-1);
        indexer.setPower(0.25);
        Actions.runBlocking(trajectory9);
        //stop intaking motor
        intake.setPower(0);
        indexer.setPower(0);
        Actions.runBlocking(trajectory10);
        //stop at in front of gate, 3 artifacts loaded for teleop. at this point there should be 9 artifacts in the classifier.
    }
}