package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "AprilTag Limelight tleop", group = "Testing")
public class LimeLightLocalizationTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    private double Tx;
    private double kP = 0.025; // TODO: Tune
    private double deadband = 0.1; // TODO: Tune
    private double turnPower;

    private double distance;

    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;

    private DcMotorEx indexer;
    private DcMotorEx intake;

    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;

    private double rx;
    private double x;
    private double y;
    private double denominator;

    private double targetRightRPM;
    private double targetLeftRPM;
    private boolean shooterOn;
    private boolean intakeOn;
    private boolean indexerOn;
    private int targetIntakePower;
    private int targetIndexerPower;

    private List<LLResultTypes.FiducialResult> tags;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // apriltag #11

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

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
    }

    @Override
    public void start() {
        limelight.start(); // can move to init if needed

        // Intake
        targetRightRPM = -1200;
        targetLeftRPM = 1200;

        shooterOn = false;
        intakeOn = false;
        indexerOn = false;

        targetIntakePower = -1;
        targetIndexerPower = -1;
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        // If sees tag, turn for tag
        if (llResult != null && llResult.isValid() && gamepad2.right_stick_button) {
            tags = llResult.getFiducialResults();

            Tx = llResult.getTx();
            turnPower = Range.clip(Tx * kP, -100, 100);

            if (turnPower <= deadband && turnPower >= -deadband) {
                turnPower = 0;
            }

            // Drive
            rx = -turnPower;

            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(y, x, rx)), 1));

            frontLeft.setPower(rx);
            backLeft.setPower(rx);
            frontRight.setPower(-rx);
            backRight.setPower(-rx);


            LLResultTypes.FiducialResult tag = tags.get(0);
            distance = tag.getRobotPoseTargetSpace().getPosition().z;

            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Distance (m)", distance);
            telemetry.addData("Tag ID", tag.getFiducialId());

            // Values are calculated from specific equations. TODO: must tune
            targetLeftRPM = 967.9 * distance / sqrt(1.4142 * distance + 0.7305);
            targetRightRPM = -967.9 * distance / sqrt(1.4142 * distance + 0.7305);

            telemetry.addData("targetLeftRPM", targetLeftRPM);
            telemetry.addData("targetRightRPM", targetRightRPM);

            ((DcMotorEx) leftShooter).setVelocity(targetLeftRPM);
            ((DcMotorEx) rightShooter).setVelocity(targetRightRPM);

            telemetry.update();

        } else {
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
        telemetry.addData("rShooterVel", ((DcMotorEx) rightShooter).getVelocity());


        telemetry.update();

    }
}