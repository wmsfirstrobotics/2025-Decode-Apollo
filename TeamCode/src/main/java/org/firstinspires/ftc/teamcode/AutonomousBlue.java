package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "AutonFirstTest", group = "Autonomous")
public class AutonomousBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(-62.5, -33, Math.toRadians(0)));

        TrajectoryActionBuilder trajectory = myBot.actionBuilder(new Pose2d(-62.5, -33, Math.toRadians(0)))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-11.5, -27, Math.toRadians(270)), Math.toRadians(270));
//                .lineToX(-47.5)
//                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
//                .waitSeconds(2)
//                .splineToLinearHeading(new Pose2d(11.5, -27, Math.toRadians(270)), Math.toRadians(270))
//                .lineToX(-47.5)
//                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
//                .waitSeconds(2)
//                .splineToLinearHeading(new Pose2d(35.5, -27, Math.toRadians(270)), Math.toRadians(270))
//                .lineToX(-47.5)
//                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
//                .waitSeconds(2);

//                .lineToX(20)
//                .turnTo(Math.toRadians(90))
//                .turnTo(Math.toRadians(0));


//        TrajectoryActionBuilder trajectory = myBot.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                .lineToX(40);

        waitForStart();

        Actions.runBlocking(trajectory.build());
    }
}
