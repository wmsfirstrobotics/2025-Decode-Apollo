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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "AutonBlue TV Side", group = "Autonomous")

public class AutonBlueTVSide extends LinearOpMode {

    private CRServo indexer;
    private DcMotor intake;
    private DcMotor leftShooter;
    private DcMotor rightShooter;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(-62.5, -39.5, Math.toRadians(0)));
        indexer = hardwareMap.get(CRServo.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        Action trajectory1 = myBot.actionBuilder(new Pose2d(-62.5, -39.5, Math.toRadians(0)))

                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(3)
                .build();
                //shoot after this

        Action trajectory2 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(54)))

                .splineToLinearHeading(new Pose2d(-15.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .build();
                // start intaking motor

        Action trajectory3 = myBot.actionBuilder(new Pose2d(-15.5, -27, Math.toRadians(270)))

                .strafeTo(new Vector2d(-11.5, -52.5))
                .build();
                // stop intaking motor

        Action trajectory4 = myBot.actionBuilder(new Pose2d(-11.5, -47.5, Math.toRadians(270)))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(3)
                .build();
                // start shooting stuff

        Action trajectory5 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(54)))

                .splineToLinearHeading(new Pose2d(11.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .build();
//start intaking motor

        Action trajectory6 = myBot.actionBuilder(new Pose2d(11.5, -27, Math.toRadians(270)))

                .strafeTo(new Vector2d(11.5, -47.5))
                .build();
        //stop intaking motor

        Action trajectory7 = myBot.actionBuilder(new Pose2d(11.5, -47.5, Math.toRadians(270)))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(3)
                .build();
        //start shooting stuff

        Action trajectory8 = myBot.actionBuilder(new Pose2d(-23.5, -23.5, Math.toRadians(54)))

                .splineToLinearHeading(new Pose2d(35.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .build();
                //start intaking motor

        Action trajectory9 = myBot.actionBuilder(new Pose2d(35.5, -27, Math.toRadians(270)))
                .strafeTo(new Vector2d(35.5, -47.5))
                .build();
                //stop intaking motor

        Action trajectory10 = myBot.actionBuilder(new Pose2d(35.5, -47.5, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(90))
                .build();
        //stop at in front of gate, 3 artifacts loaded for teleop. at this point there should be 9 artifacts in the classifier.

        waitForStart();


        Actions.runBlocking(trajectory1);
        //shoot 3 balls

        Actions.runBlocking(trajectory2);
        // start intaking motor
        intake.setPower(-1);

        Actions.runBlocking(trajectory3);
        // stop intaking motor
        intake.setPower(0);
        Actions.runBlocking(trajectory4);
        // start shooting
        Actions.runBlocking(trajectory5);
        //start intaking motor
        intake.setPower(-1);
        Actions.runBlocking(trajectory6);
        //stop intaking motor
        intake.setPower(0);
        Actions.runBlocking(trajectory7);
        //start shooting
        Actions.runBlocking(trajectory8);
        //start intaking motor
        intake.setPower(-1);
        Actions.runBlocking(trajectory9);
        //stop intaking motor
        intake.setPower(0);
        Actions.runBlocking(trajectory10);
        //stop at in front of gate, 3 artifacts loaded for teleop. at this point there should be 9 artifacts in the classifier.
    }
}
