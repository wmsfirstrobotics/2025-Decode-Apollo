package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AutonBlue TV Side States Beta", group = "Autonomous", preselectTeleOp = ("LocalizationTestBeta"))

public class AutonBlueTVSideStatesBeta extends LinearOpMode {

    private DcMotorEx indexer;
    private DcMotor intake;
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;

    private double speed = 100;
    private double intakeSpeed = 16.7;

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

                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(47)), Math.toRadians(35), new TranslationalVelConstraint(speed))
                .build();
        //shoot after this

        Action trajectory2 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(47)))

                .splineToLinearHeading(new Pose2d(-8.5, -27, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(speed))
                .build();
        // start intaking motor

        Action trajectory3 = myBot.actionBuilder(new Pose2d(-8.5, -27, Math.toRadians(270)))

                .strafeTo(new Vector2d(-8.5, -50.5), new TranslationalVelConstraint(intakeSpeed))
                .strafeTo(new Vector2d(1, -50.5), new TranslationalVelConstraint(speed))
                .strafeTo(new Vector2d(1, -53.5), new TranslationalVelConstraint(speed))
                .build();
        // stop intaking motor

        Action trajectory4 = myBot.actionBuilder(new Pose2d(0, -53, Math.toRadians(270)))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(50)), Math.toRadians(-200), new TranslationalVelConstraint(speed))
                .build();
        // start shooting stuff

        Action trajectory5 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(48)))
                .setTangent(Math.toRadians(22.5))
                .splineToSplineHeading(new Pose2d(24.5, -27, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(speed))
                .strafeTo(new Vector2d(24.5, -50.5), new TranslationalVelConstraint(intakeSpeed))
                .build();
//start intaking motor

        //stop intaking motor

        Action trajectory7 = myBot.actionBuilder(new Pose2d(24.5, -50.5, Math.toRadians(270)))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-19.5, -27.5, Math.toRadians(49)), Math.toRadians(-200), new TranslationalVelConstraint(speed))
                .build();
        //start shooting stuff

//        Action trajectory8 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(47)))
//
//                .splineToLinearHeading(new Pose2d(39.5, -27, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(speed))
//                .build();
//        //start intaking motor
//
//        Action trajectory9 = myBot.actionBuilder(new Pose2d(39.5, -27, Math.toRadians(270)))
//                .strafeTo(new Vector2d(39.5, -50.5), new TranslationalVelConstraint(intakeSpeed))
//                .build();
//        //stop intaking motor

        Action trajectory8new = myBot.actionBuilder(new Pose2d(-19.5, -27.5, Math.toRadians(47)))
                .setTangent(Math.toRadians(345))
                .splineToSplineHeading(new Pose2d(51.5, -27, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(speed))
                .splineToSplineHeading(new Pose2d(51.5, -50.5, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(intakeSpeed))
                .build();

        Action trajectory10 = myBot.actionBuilder(new Pose2d(51.5, -50.5, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-18.5, -28.5, Math.toRadians(45)), Math.toRadians(-200), new TranslationalVelConstraint(speed))
                .build();
        //stop at in front of gate, 3 artifacts loaded for teleop. at this point there should be 9 artifacts in the classifier.
        Action trajectory11beta = myBot.actionBuilder(new Pose2d(-18.5, -28.5, Math.toRadians(45)))
                .strafeTo(new Vector2d(12, -58.5))
                .build();

        waitForStart();

        leftShooter.setVelocity(925);
        rightShooter.setVelocity(-925);
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
        intake.setPower(-1);
        indexer.setPower(0.25);

        Actions.runBlocking(trajectory5);
        //start intaking motor

//        Actions.runBlocking(trajectory6);
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
//        indexer.setPower(0);
//        intake.setPower(0);


//        Actions.runBlocking(trajectory8);
        //start intaking motor
        intake.setPower(-1);
        indexer.setPower(0.25);
//        Actions.runBlocking(trajectory9);
        Actions.runBlocking(trajectory8new);
        //stop intaking motor
        intake.setPower(0);
        indexer.setPower(0);
        Actions.runBlocking(trajectory10);
        indexer.setPower(-1);
        intake.setPower(-0.5);
        sleep(900);
        intake.setPower(-1);
        sleep(700);
        indexer.setPower(0);
        intake.setPower(0);

        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
        Actions.runBlocking(trajectory11beta);
        //stop at in front of gate, 3 artifacts loaded for teleop. at this point there should be 9 artifacts in the classifier.

    }
}