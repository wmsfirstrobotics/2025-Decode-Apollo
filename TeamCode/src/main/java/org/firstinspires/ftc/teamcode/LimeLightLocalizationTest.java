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

@TeleOp(name = "use this for LL", group = "Testing")
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

    LLResultTypes.FiducialResult tag;

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
        // Get LL data
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        // Drive
        y = Math.pow(gamepad1.left_stick_y, 3);
        x = Math.pow(-gamepad1.left_stick_x * 1.1, 3);

        // if tag && key, set angle to ignore joystick and do LL

        if (llResult != null && llResult.isValid() && gamepad1.b) {

            Tx = llResult.getTx();
            turnPower = Range.clip(Tx * kP, -1, 1);

            if (turnPower <= deadband && turnPower >= -deadband) {
                turnPower = 0;
            }

            rx = -turnPower;

            distance = getDistance(llResult.getTa());

        } else {
            rx = -gamepad1.right_stick_x;

            distance = -1;
        }

        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(y, x, rx)), 1));

        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower(((y - x) + rx) / denominator);
        frontRight.setPower(((y - x) - rx) / denominator);
        backRight.setPower(((y + x) - rx) / denominator);


        // process shooting/intaking/indexing



        // Values are calculated from specific equations. TODO: must tune



        // Shooters
        if (gamepad2.bWasPressed()) {
            shooterOn = !shooterOn;
        }

        if (shooterOn) {
            if (distance != -1) {
                targetLeftRPM = 96.79 * distance / sqrt(1.4142 * distance + 0.7305);
                targetRightRPM = -96.79 * distance / sqrt(1.4142 * distance + 0.7305);

            } else if (gamepad2.dpadUpWasPressed()) {
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

        telemetry.addData("ta", llResult.getTa());
        telemetry.addData("targetLeftRPM", targetLeftRPM);
        telemetry.addData("targetRightRPM", targetRightRPM);
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("Distance (in)", distance);

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

//        telemetry.addData("backLeft", backLeft.getCurrentPosition());
//        telemetry.addData("backRight", backRight.getCurrentPosition());
//        telemetry.addData("frontRight", frontRight.getCurrentPosition());
//        telemetry.addData("frontLeft", frontLeft.getCurrentPosition());

        telemetry.addData("LShooterVel", ((DcMotorEx) leftShooter).getVelocity());
        telemetry.addData("rShooterVel", ((DcMotorEx) rightShooter).getVelocity());


        telemetry.update();

    }

    public double getDistance(double ta) {
        double scale = 30665.95;

        return (scale * 0.394 / (ta * 100));
    }
}