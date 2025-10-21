package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "limelight auto-detect servo", group = "Testing")
public class LimelightNew extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    // hardware references
    private Servo posServo = null;
    private CRServo crServo = null;

    // config
    private static final String SERVO_NAME = "limelightservo";
    private static final double START_POS = 0.5;

    // ADJUSTED: Reduced step size for smoother movement
    private static final double STEP_POS = 0.015;

    // ADJUSTED: Increased deadband to stop oscillation (hunting)
    private static final double CENTER_BAND_DEG = 6.0;

    // ADJUSTED: Reduced gain for less aggressive CR servo movement
    private static final double CR_KP = 0.01;

    private static final long CAL_STEP_MS = 350;
    private static final double CAL_EPS = 0.08;

    // state machine for calibration / detection
    private enum State { INIT, CAL_SET0, CAL_WAIT0, CAL_SET1, CAL_WAIT1, CAL_SETC, CAL_WAITC, CAL_OK, CAL_FAIL_DETECT_CR, USING_POS, USING_CR, SAFE_MODE }
    private State state = State.INIT;
    private long stateTs = 0;

    // control state
    private double storedPos = START_POS;
    private long lastMoveTs = 0;

    // ADJUSTED: Increased interval for slower positional movement rate
    private static final long MOVE_INTERVAL_MS = 250;
    private boolean panic = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // attempt to bind both types (posServo first)
        try { posServo = hardwareMap.get(Servo.class, SERVO_NAME); } catch (Exception e) { posServo = null; }
        try { crServo = hardwareMap.get(CRServo.class, SERVO_NAME); } catch (Exception e) { crServo = null; }

        // if posServo present, set to center (we'll calibrate in start())
        if (posServo != null) {
            try { posServo.setPosition(START_POS); } catch (Exception ignored) { }
        }
        // ensure crServo power 0 if present
        if (crServo != null) {
            try { crServo.setPower(0.0); } catch (Exception ignored) { }
        }

        state = State.CAL_SET0;
        stateTs = System.currentTimeMillis();
       //telemetry.addLine("init: starting auto-detect cal");
        //telemetry.addData("posServo", posServo != null ? "present" : "none");
       /// telemetry.addData("crServo", crServo != null ? "present" : "none");
        telemetry.update();

        // imu optional, try bind but it's not needed for detection
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot rev = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            );
            imu.initialize(new IMU.Parameters(rev));
        } catch (Exception ignored) { imu = null; }
    }

    @Override
    public void start() {
        limelight.start();
        stateTs = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // panic button -> immediate safe mode
        if (gamepad1.b) {
            panic = true;
            state = State.SAFE_MODE;
        }

        // state machine for calibration and detection (non-blocking)
        switch (state) {
            case CAL_SET0:
                telemetry.addLine("cal: set 0.0");
                if (posServo != null) {
                    safePosSet(0.0);
                }
                state = State.CAL_WAIT0;
                stateTs = now;
                break;

            case CAL_WAIT0:
                telemetry.addLine("cal: wait0");
                if (now - stateTs >= CAL_STEP_MS) {
                    state = State.CAL_SET1;
                    stateTs = now;
                }
                break;

            case CAL_SET1:
                telemetry.addLine("cal: set 1.0");
                if (posServo != null) safePosSet(1.0);
                state = State.CAL_WAIT1;
                stateTs = now;
                break;

            case CAL_WAIT1:
                telemetry.addLine("cal: wait1");
                if (now - stateTs >= CAL_STEP_MS) {
                    state = State.CAL_SETC;
                    stateTs = now;
                }
                break;

            case CAL_SETC:
                telemetry.addLine("cal: set center");
                if (posServo != null) safePosSet(START_POS);
                state = State.CAL_WAITC;
                stateTs = now;
                break;

            case CAL_WAITC:
                telemetry.addLine("cal: waitc");
                if (now - stateTs >= CAL_STEP_MS) {
                    // evaluate calibration if posServo existed
                    if (posServo != null) {
                        double r0 = safeReadPos(); // reported after center (best-effort)
                        // we can't read previous values easily here; instead check that reported pos is reasonable near center
                        if (!Double.isNaN(r0) && Math.abs(r0 - START_POS) <= CAL_EPS) {
                            // looks ok: positional servo reports center as commanded -> use positional
                            state = State.CAL_OK;
                        } else {
                            // reported value not matching center -> try detect cr fallback
                            state = State.CAL_FAIL_DETECT_CR;
                        }
                    } else {
                        // no positional servo bound, try CR
                        state = State.CAL_FAIL_DETECT_CR;
                    }
                }
                break;

            case CAL_OK:
                telemetry.addLine("cal: ok -> using positional servo");
                storedPos = START_POS;
                lastMoveTs = now;
                state = State.USING_POS;
                break;

            case CAL_FAIL_DETECT_CR:
                telemetry.addLine("cal: failed or pos not behaving");
                // if cr servo exists, switch to cr mode
                if (crServo != null) {
                    telemetry.addLine("switching to crservo control");
                    // ensure cr stopped
                    try { crServo.setPower(0.0); } catch (Exception ignored) {}
                    state = State.USING_CR;
                } else {
                    telemetry.addLine("no crservo available -> safe mode");
                    state = State.SAFE_MODE;
                }
                break;

            case USING_POS:
                // normal positional tracking: single steps only, rate-limited
                doPosTracking(now);
                break;

            case USING_CR:
                // normal cr tracking: apply power while outside center band
                doCrTracking();
                break;

            case SAFE_MODE:
                // stop everything
                try { if (crServo != null) crServo.setPower(0.0); } catch (Exception ignored) {}
                // Reset positional servo to center/stop if possible
                try { if (posServo != null) safePosSet(START_POS); } catch (Exception ignored) {}
                telemetry.addLine("SAFE MODE: not moving");
                telemetry.addLine("check wiring, controller config, or replace servo");
                break;
        }

        // telemetry: always print some helpful info
        //telemetry.addData("state", state.toString());
       // telemetry.addData("panic", panic);
        if (posServo != null) {
            double rp = safeReadPos();
          //  telemetry.addData("posServo.report", Double.isNaN(rp) ? "nan" : String.format("%.3f", rp));
        }
        if (crServo != null) {
            double p = safeReadCrPower();
            telemetry.addData("crServo.power", Double.isNaN(p) ? "nan" : String.format("%.3f", p));
        }
        telemetry.update();
    }

    // helper: non-throwing setPosition
    private void safePosSet(double p) {
        try { posServo.setPosition(Range.clip(p, 0.0, 1.0)); } catch (Exception ignored) {}
    }

    private double safeReadPos() {
        if (posServo == null) return Double.NaN;
        try { return posServo.getPosition(); } catch (Exception e) { return Double.NaN; }
    }

    private double safeReadCrPower() {
        if (crServo == null) return Double.NaN;
        try { return crServo.getPower(); } catch (Exception e) { return Double.NaN; }
    }

    // positional servo tracking (single step per interval)
    private void doPosTracking(long now) {
        if (panic) { state = State.SAFE_MODE; return; }

        LLResult r = limelight.getLatestResult();
        boolean moved = false;
        boolean tagFound = false;

        if (r != null) {
            List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
            if (f != null && !f.isEmpty()) {
                // --- TAG IS FOUND: Execute Tracking Logic ---
                tagFound = true;
                double tx = f.get(0).getTargetXDegrees();
                telemetry.addData("fid", f.get(0).getFiducialId());
                telemetry.addData("tx", tx);

                if (Math.abs(tx) > CENTER_BAND_DEG && (now - lastMoveTs) >= MOVE_INTERVAL_MS) {
                    // tx > 0 (tag is right) -> need to move camera right (decreasing storedPos)
                    // This direction (1.0 : -1.0) was the one that worked for tracking.
                    double dir = (tx > 0) ? 1.0 : -1.0;
                    storedPos = Range.clip(storedPos + dir * STEP_POS, 0.0, 1.0);
                    safePosSet(storedPos);
                    lastMoveTs = now;
                    moved = true;
                }
            }
        }

        // --- If no tag is found, gradually return to START_POS (center) ---
        if (!tagFound) {
            if (storedPos != START_POS && (now - lastMoveTs) >= MOVE_INTERVAL_MS) {
                double delta = START_POS - storedPos;

                if (Math.abs(delta) < STEP_POS) {
                    storedPos = START_POS;
                } else {
                    double dir = Math.signum(delta);
                    storedPos = Range.clip(storedPos + dir * STEP_POS, 0.0, 1.0);
                }

                safePosSet(storedPos);
                lastMoveTs = now;
                telemetry.addLine("pos: no fiducial - returning to center");
                moved = true;
            } else {
                telemetry.addLine("pos: no fiducial - holding center/waiting");
            }
        }

       // telemetry.addData("posMoved", moved);
       // telemetry.addData("storedPos", storedPos);
    }

    // cr servo tracking: apply power while tag outside band; stop otherwise
    private void doCrTracking() {
        if (panic) { state = State.SAFE_MODE; return; }

        double power = 0.0;
        LLResult r = limelight.getLatestResult();

        if (r != null) {
            List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
            if (f != null && !f.isEmpty()) {
                double tx = f.get(0).getTargetXDegrees();
                telemetry.addData("fid", f.get(0).getFiducialId());
                telemetry.addData("tx", tx);

                if (Math.abs(tx) > CENTER_BAND_DEG) {
                    // Negative sign restored, as it was the direction that worked for tracking.
                    power = -Range.clip(tx * CR_KP, -1.0, 1.0);
                } else {
                    // Tag is within the CENTER_BAND_DEG -> STOP
                    power = 0.0;
                }
            } else {
                // NO TAG IN VIEW -> STOP SERVO
                telemetry.addLine("cr: no fiducial - stopping");
                power = 0.0;
            }
        } else {
            // NO LIMELIGHT RESULT -> STOP SERVO
            telemetry.addLine("cr: no limelight result - stopping");
            power = 0.0;
        }

        try { crServo.setPower(power); } catch (Exception ignored) {}
        telemetry.addData("crPowerSet", power);
    }
}