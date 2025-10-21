package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "limelight auto-detect servo", group = "Testing")
public class LimelightNew extends OpMode {
    private Limelight3A limelight;

    private Servo posServo = null;
    private CRServo crServo = null;

    private static final Map<Integer, String> TAG_NAMES = new HashMap<Integer, String>() {{
        put(21, "GPP");
        put(22, "PGP");
        put(23, "PPG");
        put(20, "Blue Goal");
        put(24, "Red Goal");
    }};


    private static final String SERVO_NAME = "limelightservo";
    private static final double START_POS = 0.5;


    private static final double STEP_POS = 0.005;
    private static final double CENTER_BAND_DEG = 6.0;
    private static final double CR_KP = 0.005;


    private static final long CAL_STEP_MS = 350;
    private static final double CAL_EPS = 0.08;


    private enum State { INIT, CAL_SET0, CAL_WAIT0, CAL_SET1, CAL_WAIT1, CAL_SETC, CAL_WAITC, CAL_OK, CAL_FAIL_DETECT_CR, USING_POS, USING_CR, SAFE_MODE }
    private State state = State.INIT;
    private long stateTs = 0;


    private double storedPos = START_POS;
    private long lastMoveTs = 0;
    private static final long MOVE_INTERVAL_MS = 250;
    private boolean panic = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);


        try { posServo = hardwareMap.get(Servo.class, SERVO_NAME); } catch (Exception e) { posServo = null; }
        try { crServo = hardwareMap.get(CRServo.class, SERVO_NAME); } catch (Exception e) { crServo = null; }

        if (posServo != null) {
            try { posServo.setPosition(START_POS); } catch (Exception ignored) { }
        }
        if (crServo != null) {
            try { crServo.setPower(0.0); } catch (Exception ignored) { }
        }

        state = State.CAL_SET0;
        stateTs = System.currentTimeMillis();
        telemetry.addLine("Limelight: Starting auto-detect cal");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        stateTs = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        if (gamepad1.b) {
            panic = true;
            state = State.SAFE_MODE;
        }

        switch (state) {
            case CAL_SET0:
                if (posServo != null) safePosSet(0.0);
                state = State.CAL_WAIT0;
                stateTs = now;
                break;
            case CAL_WAIT0:
                if (now - stateTs >= CAL_STEP_MS) {
                    state = State.CAL_SET1;
                    stateTs = now;
                }
                break;
            case CAL_SET1:
                if (posServo != null) safePosSet(1.0);
                state = State.CAL_WAIT1;
                stateTs = now;
                break;
            case CAL_WAIT1:
                if (now - stateTs >= CAL_STEP_MS) {
                    state = State.CAL_SETC;
                    stateTs = now;
                }
                break;
            case CAL_SETC:
                if (posServo != null) safePosSet(START_POS);
                state = State.CAL_WAITC;
                stateTs = now;
                break;
            case CAL_WAITC:
                if (now - stateTs >= CAL_STEP_MS) {
                    if (posServo != null) {
                        double r0 = safeReadPos();
                        if (!Double.isNaN(r0) && Math.abs(r0 - START_POS) <= CAL_EPS) {
                            state = State.CAL_OK;
                        } else {
                            state = State.CAL_FAIL_DETECT_CR;
                        }
                    } else {
                        state = State.CAL_FAIL_DETECT_CR;
                    }
                }
                break;
            case CAL_OK:
                storedPos = START_POS;
                lastMoveTs = now;
                state = State.USING_POS;
                break;
            case CAL_FAIL_DETECT_CR:
                if (crServo != null) {
                    try { crServo.setPower(0.0); } catch (Exception ignored) {}
                    state = State.USING_CR;
                } else {
                    state = State.SAFE_MODE;
                }
                break;
            case USING_POS:
                doPosTracking(now);
                break;
            case USING_CR:
                doCrTracking();
                break;
            case SAFE_MODE:
                try { if (crServo != null) crServo.setPower(0.0); } catch (Exception ignored) {}
                try { if (posServo != null) safePosSet(START_POS); } catch (Exception ignored) {}
                break;
        }


        telemetry.addData("State", state.toString());

        LLResult r = limelight.getLatestResult();
        if (r != null) {
            List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
            if (f != null && !f.isEmpty()) {
                int currentTagId = f.get(0).getFiducialId();
                double tx = f.get(0).getTargetXDegrees();


                String tagName = TAG_NAMES.getOrDefault(currentTagId, "Unknown ID: " + currentTagId);

                telemetry.addData("--- TAG DETECTED ---", "");
                telemetry.addData("Tag ID", currentTagId);
                telemetry.addData("Object", tagName);
                telemetry.addData("TX (Error)", String.format("%.2f deg", tx));
            } else {
                telemetry.addLine("TX (Error): No Tag Visible (Servo Stopped)");
            }
        } else {
            telemetry.addLine("TX (Error): No Limelight Result (Check Connection)");
        }

        telemetry.update();
    }

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

    private void doPosTracking(long now) {
        if (panic) { state = State.SAFE_MODE; return; }

        LLResult r = limelight.getLatestResult();
        boolean tagFound = false;

        if (r != null) {
            List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
            if (f != null && !f.isEmpty()) {
                tagFound = true;
                double tx = f.get(0).getTargetXDegrees();

                if (Math.abs(tx) > CENTER_BAND_DEG && (now - lastMoveTs) >= MOVE_INTERVAL_MS) {

                    double dir = (tx > 0) ? 1.0 : -1.0;
                    storedPos = Range.clip(storedPos + dir * STEP_POS, 0.0, 1.0);
                    safePosSet(storedPos);
                    lastMoveTs = now;
                }
            }
        }


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
            }
        }
    }

    private void doCrTracking() {
        if (panic) { state = State.SAFE_MODE; return; }

        double power = 0.0;
        LLResult r = limelight.getLatestResult();

        if (r != null) {
            List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
            if (f != null && !f.isEmpty()) {
                double tx = f.get(0).getTargetXDegrees();

                if (Math.abs(tx) > CENTER_BAND_DEG) {

                    power = -Range.clip(tx * CR_KP, -1.0, 1.0);
                } else {

                    power = 0.0;
                }
            } else {

                power = 0.0;
            }
        } else {

            power = 0.0;
        }

        try { crServo.setPower(power); } catch (Exception ignored) {}
    }
}