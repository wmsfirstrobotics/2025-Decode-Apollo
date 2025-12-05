package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red HP Side (Forwards)", group = "Autonomous")

public class AutonRedHPSideFORWARD extends LinearOpMode {

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
        MecanumDrive myBot = new MecanumDrive(hardwareMap, new Pose2d(62.5, 18, Math.toRadians(180)));

        Action trajectory = myBot.actionBuilder(new Pose2d(62.5, 18, Math.toRadians(180)))

                .strafeTo(new Vector2d(75, 18))
                .build();
        waitForStart();

//        FakeIndexServo fakeIndexServo = new FakeIndexServo(hardwareMap);

        Actions.runBlocking(trajectory);;
    }
}
