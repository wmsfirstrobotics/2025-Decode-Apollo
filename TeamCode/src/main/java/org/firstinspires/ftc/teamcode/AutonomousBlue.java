package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

@Autonomous(name = "AutonBlue", group = "Autonomous")

public class AutonomousBlue extends LinearOpMode {

//    public class FakeIndexServo {
//        private CRServo servo;
//        public FakeIndexServo(HardwareMap hardwareMap) {
//            servo = hardwareMap.get(CRServo.class, "servo");
//        }
//
//        public class FakeIndexerSpin implements Action {
//            public boolean run (@NonNull TelemetryPacket packet){
//                servo.set(1);
//                return false;
//            }
//
//        }
//        public Action fakeIndexerSpin() {
//            return new FakeIndexerSpin();
//        }
//
//}


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(62.5, -24, Math.toRadians(180)));

        Action trajectory = myBot.actionBuilder(new Pose2d(62.5, -24, Math.toRadians(180)))

              //.splineToLinearHeading(new Pose2d(-7,-58, Math.toRadians(180)), Math.toRadians(270))
              //.lineToY(-40)
                .splineToLinearHeading(new Pose2d(-11.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .strafeTo(new Vector2d(-11.5, -47.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(2)
         
                .splineToLinearHeading(new Pose2d(11.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .strafeTo(new Vector2d(11.5, -47.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(35.5, -27, Math.toRadians(270)), Math.toRadians(270))
                .strafeTo(new Vector2d(35.5, -47.5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23.5, -23.5, Math.toRadians(54)), Math.toRadians(-200))
                .waitSeconds(2)
                .build();
        waitForStart();

//        FakeIndexServo fakeIndexServo = new FakeIndexServo(hardwareMap);

        Actions.runBlocking(trajectory);;
    }
}
