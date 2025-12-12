package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red HP Side (Sideways)", group = "Autonomous", preselectTeleOp = ("LiLiLocalizationTest"))

public class AutonRedHPSideSIDEWAYS extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(62.5, 18, Math.toRadians(180)));

        Action trajectory = myBot.actionBuilder(new Pose2d(62.5, 18, Math.toRadians(180)))

                .strafeTo(new Vector2d(75, 34))
                .build();
        waitForStart();

//        FakeIndexServo fakeIndexServo = new FakeIndexServo(hardwareMap);

        Actions.runBlocking(trajectory);;
    }
}
