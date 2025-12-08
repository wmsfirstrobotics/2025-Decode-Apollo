package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Test TeleOp", group = "TeleOp")
public class AutoAimTeleOpBlue extends LinearOpMode {

    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx indexer;
    private DcMotorEx intake;


    @Override
    public void runOpMode() {
        double targetRightRPM;
        double targetLeftRPM;
        boolean shooterOn;
        boolean intakeOn;
        boolean indexerOn;
        int targetIntakePower;
        int targetIndexerPower;
        float y;
        double x;
        float rx;
        double denominator;

        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        ((DcMotorEx) leftShooter).setVelocityPIDFCoefficients(60, 0, 0, 13.2);
        ((DcMotorEx) rightShooter).setVelocityPIDFCoefficients(60, 0, 0, 13.2);


        waitForStart();
        if (opModeIsActive()) {

            // Intake
            targetRightRPM = -1200;
            targetLeftRPM = 1200;

            shooterOn = false;
            intakeOn = false;
            indexerOn = false;

            targetIntakePower = -1;
            targetIndexerPower = -1;


            while (opModeIsActive()) {

                // Drive
                y = gamepad1.left_stick_y;
                x = -(gamepad1.left_stick_x * 1.1);
                rx = -gamepad1.right_stick_x;

                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(y, x, rx)), 1));

                frontLeft.setPower((y + x + rx) / denominator);
                backLeft.setPower(((y - x) + rx) / denominator);
                frontRight.setPower(((y - x) - rx) / denominator);
                backRight.setPower(((y + x) - rx) / denominator);


                // Shooters
                if (gamepad2.bWasPressed()) {
                    shooterOn = !shooterOn;
                }

                if (shooterOn) {
                    if (gamepad2.dpadUpWasPressed()) {
                        targetRightRPM = -1172.5;
                        targetLeftRPM = 1172.5;

                    } else if (gamepad2.dpadDownWasPressed()) {
                        targetRightRPM = -957;
                        targetLeftRPM = 957;

                    } else if (gamepad2.leftStickButtonWasPressed()) {
                        targetRightRPM = -1075;
                        targetLeftRPM = 1075;
                    }
                    ((DcMotorEx) leftShooter).setVelocity(targetLeftRPM);
                    ((DcMotorEx) rightShooter).setVelocity(targetRightRPM);

                } else {
                    ((DcMotorEx) leftShooter).setVelocity(0);
                    ((DcMotorEx) rightShooter).setVelocity(0);
                }


                // Indexer
                if (gamepad2.aWasPressed()) {
                    indexerOn = !indexerOn;
                }

                if (indexerOn) {
                    if (gamepad1.dpadLeftWasPressed()) {
                        targetIndexerPower = -1;

                    } else if (gamepad1.dpadRightWasPressed()) {
                        targetIndexerPower = 1;
                    }
                    indexer.setPower(targetIndexerPower);

                } else {
                    indexer.setPower(0);
                }


                // Intake
                if (gamepad2.yWasPressed()) {
                    intakeOn = !intakeOn;
                }

                if (intakeOn) {
                    if (gamepad2.leftBumperWasPressed()) {
                        targetIntakePower = -1;

                    } else if (gamepad2.rightBumperWasPressed()) {
                        targetIntakePower = 1;
                    }
                    intake.setPower(targetIntakePower);

                } else {
                    intake.setPower(0);
                }


                // ShooterTel
                if (((DcMotorEx) leftShooter).getVelocity() < 700) {
                    telemetry.addData("ShooterRPM", "Off");

                } else {
                    if (targetLeftRPM == 1162.5) {
                        telemetry.addData("ShooterRPM", ((DcMotorEx) backLeft).getVelocity());

                    } else if (targetLeftRPM == 1075) {
                        telemetry.addData("ShooterRPM", ((DcMotorEx) backLeft).getVelocity());

                    } else if (targetLeftRPM == 950) {
                        telemetry.addData("ShooterRPM", ((DcMotorEx) backLeft).getVelocity());
                    }
                }


                // IntakeTel
                if (intakeOn == false) {
                    telemetry.addData("IntakeDIR", "Off");
                } else if (targetIntakePower == 1) {
                    telemetry.addData("IntakeDIR", "In");
                } else if (targetIntakePower == -1) {
                    telemetry.addData("IntakeDIR", "Out");
                } else if (intakeOn && intake.getPower() == 0) {
                    telemetry.addData("IntakeDIR", "On; No Target Power");
                }


                // IndexerTel
                if (indexer.getPower() == 0) {
                    telemetry.addData("IndexerDIR", "Off");
                } else if (indexer.getPower() < 0) {
                    telemetry.addData("IndexerDIR", "Out");
                } else if (indexer.getPower() > 0) {
                    telemetry.addData("IndexerDIR", "In");
                } else if (indexerOn && indexer.getPower() == 0) {
                    telemetry.addData("IndexerDIR", "On; No Target Power");
                }

                telemetry.addData("backLeft", backLeft.getCurrentPosition());
                telemetry.addData("backRight", backRight.getCurrentPosition());
                telemetry.addData("frontRight", frontRight.getCurrentPosition());
                telemetry.addData("frontLeft", frontLeft.getCurrentPosition());

                telemetry.addData("LShooterVel", ((DcMotorEx) leftShooter).getVelocity());
                telemetry.addData("RShooterVel", ((DcMotorEx) rightShooter).getVelocity());


                telemetry.update();
            }
        }
    }
}