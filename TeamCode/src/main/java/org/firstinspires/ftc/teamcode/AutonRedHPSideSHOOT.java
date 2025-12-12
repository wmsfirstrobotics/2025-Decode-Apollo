package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Red HP Side (Shooting)", group = "Autonomous", preselectTeleOp = ("LiLiLocalizationTest"))

public class AutonRedHPSideSHOOT extends LinearOpMode {

    private DcMotorEx indexer;
    private DcMotor intake;
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(62.5, 18, Math.toRadians(0)));
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        Action trajectory1 = myBot.actionBuilder(new Pose2d(62.5, 18, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(53, 16, Math.toRadians(225)), Math.toRadians(180))
                //shoot
                .build();

        Action trajectory2 = myBot.actionBuilder(new Pose2d(53, 16, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(60,50,Math.toRadians(180)),Math.toRadians(315))
                .build();


        waitForStart();

        leftShooter.setVelocity(1150);
        rightShooter.setVelocity(-1150);
        Actions.runBlocking(trajectory1);
        indexer.setPower(-1);
        intake.setPower(-0.5);
        sleep(435);
        intake.setPower(-1);
        sleep(700);
        indexer.setPower(0);
        intake.setPower(0);
        Actions.runBlocking(trajectory2);
    }
}
