package org.firstinspires.ftc.teamcode.teleop.stage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Dual Motor Test - Intake and Shooting + goBILDA Servo support + cb_dcMotor
 * Motor1 - Ball Intake Motor
 * Motor2/3 - Ball Shooting Motors
 * Motor4 - cb_dcMotor (runs with shooting motors)
 * Optional: goBILDA standard positional Servo (name: "servo1")
 * // Optional: goBILDA CR Servo (continuous rotation, name: "gb_crservo") // commented out
 //*
 * INTAKE CONTROLS (Motor1):
 * - A Button: Toggle intake motor ON/OFF
 * - DPAD Left/Right: Change intake direction
 //*
 * SHOOTING CONTROLS (Motor2/3/4):
 * - B Button: Toggle shooting motors and cb_dcMotor ON/OFF
 * - DPAD Up/Down: Increase/Decrease shooting power
 * - X Button: Set shooting to 75% power
 * - Y Button: Set shooting to 100% power
 * - Right Bumper: Fine control (2.5%)
 * - Left Bumper: Normal control (5%)
 //*
 * SERVO CONTROLS (goBILDA):
 * - Positional Servo (servo1):
 *   - Right Trigger: increase position
 *   - Left  Trigger: decrease position
 *   - Left Stick Button: set to 0.00 (min)
 *   - Right Stick Button: set to 1.00 (max)
 * // - CR Servo (gb_crservo):
 * //   - Power = (Right Trigger - Left Trigger)
 */
@TeleOp(name = "VV17181 Teleop", group = "Test")
public class TestDrive17181Main extends LinearOpMode {

    // DC motor declarations
    private DcMotorEx im_intakeMotor;
    private DcMotorEx sm_shootingMotor1;
    private DcMotorEx sm_shootingMotor2;
    private DcMotorEx cb_dcMotor; // Conveyor Belt motor running with shooting motors

    // Servo declarations (optional — mapped if present in hardware config)
    private Servo gb_servo;        // standard positional servo (0.0 - 1.0)
    // private CRServo gb_crServo;    // continuous-rotation servo (-1.0 to 1.0 power) // commented out
    private boolean gb_hasServo = false;
    // private boolean gb_hasCRServo = false; // commented out

    // Timing and measurement
    private ElapsedTime runtime = new ElapsedTime();

    // Motor specifications constants
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double BARE_MOTOR_RPM = 5800.0;  // 5000 Series bare motor speed
    private static final double GEAR_RATIO = 1.0;         // 1:1 gear ratio (direct drive)
    private static final double FREE_RUN_RPM = BARE_MOTOR_RPM / GEAR_RATIO;  // Output shaft: 5800 RPM
    private static final int ENCODER_CPR = 537; // Since gear ratio is 1:1, use motor's native CPR

    // Intake motor variables (Motor1)
    private final double im_intakePower = 0.9; // Fixed at 90% for intake
    private boolean im_intakeRunning = false;
    private boolean im_intakeForward = true;

    // Shooting motor variables (Motor2/3/4)
    private double sm_shootingPower = 0.5; // Variable power for shooting
    private final double cb_shootingPower = 0.75; // Variable power for Conveyor Belt Motor
    private double sm_shootingPowerIncrement = 0.05;
    private boolean sm_shootingRunning = false;

    // goBILDA positional servo state
    private double gb_servoPos = 0.00;     // center by default
    private static final double GB_SERVO_STEP = 0.02; // step per tick when trigger held
    private static final double GB_SERVO_MIN_ANGLE = 0.0;   // degrees
    private static final double GB_SERVO_MAX_ANGLE = 180.0; // degrees

    // Button state tracking
    private boolean im_lastAState = false;
    private boolean sm_lastBState = false;
    private boolean sm_lastUpState = false;
    private boolean sm_lastDownState = false;
    private boolean im_lastLeftState = false;
    private boolean im_lastRightState = false;

    // Motor performance tracking
    private double im_maxIntakeRPM = 0;
    private double sm_maxShootingRPM1 = 0;
    private double sm_maxShootingRPM2 = 0;
    private double cb_maxRPM = 0;
    private double im_avgIntakeRPM = 0;
    private double sm_avgShootingRPM1 = 0;
    private double sm_avgShootingRPM2 = 0;
    private double cb_avgRPM = 0;
    private int im_intakeSampleCount = 0;
    private int sm_shootingSampleCount1 = 0;
    private int sm_shootingSampleCount2 = 0;
    private int cb_sampleCount = 0;

    @Override
    public void runOpMode() {
        // Initialize motors/servos
        initializeMotorsAndServos();

        // Display pre-test information
        displayMotorSpecifications();
        displayControls();

        telemetry.addData("Status", "Initialized - Ready for testing");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Main testing loop
        while (opModeIsActive()) {
            // Check for all control inputs
            checkIntakeControls();
            checkShootingControls();
            checkServoControls();

            // Apply power to intake motor (Motor1)
            if (im_intakeRunning) {
                double appliedIntakePower = im_intakeForward ? im_intakePower : -im_intakePower;
                im_intakeMotor.setPower(appliedIntakePower);
            } else {
                im_intakeMotor.setPower(0);
            }

            // Apply power to all shooting motors (Motor2/3/4)
            if (sm_shootingRunning) {
                sm_shootingMotor1.setPower(sm_shootingPower);
                sm_shootingMotor2.setPower(sm_shootingPower);
                cb_dcMotor.setPower(cb_shootingPower);
            } else {
                sm_shootingMotor1.setPower(0);
                sm_shootingMotor2.setPower(0);
                cb_dcMotor.setPower(0);
            }

            // Apply servo outputs (if present)
            if (gb_hasServo) {
                gb_servo.setPosition(gb_servoPos);
            }
            // (CR servo power is set directly in checkServoControls()) // commented out

            // Monitor and display real-time performance
            updateMotorTelemetry();

            // Small delay
            sleep(20);
        }

        // Stop actuators when opmode ends
        im_intakeMotor.setPower(0);
        sm_shootingMotor1.setPower(0);
        sm_shootingMotor2.setPower(0);
        cb_dcMotor.setPower(0);
    }

    private void initializeMotorsAndServos() {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        try {
            // DC motors
            im_intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            sm_shootingMotor1 = hardwareMap.get(DcMotorEx.class, "sm1");
            sm_shootingMotor2 = hardwareMap.get(DcMotorEx.class, "sm2");
            cb_dcMotor = hardwareMap.get(DcMotorEx.class, "cbMotor");

            // Optional servos — try to map; skip if not in config
            try {
                gb_servo = hardwareMap.get(Servo.class, "sm_servo");
                gb_hasServo = true;
            } catch (Exception ignored) { gb_hasServo = false; }

            // Directions
            im_intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            sm_shootingMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
            sm_shootingMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            cb_dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Encoders & modes
            im_intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sm_shootingMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sm_shootingMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cb_dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            im_intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sm_shootingMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sm_shootingMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            cb_dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Zero power behaviors
            im_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sm_shootingMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sm_shootingMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cb_dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Init servo positions/power
            if (gb_hasServo) gb_servo.setPosition(gb_servoPos);
            // if (gb_hasCRServo) gb_crServo.setPower(0); // commented out

            telemetry.addData("Status", "Hardware initialized successfully");
            telemetry.addData("Servo (servo1)", gb_hasServo ? "FOUND" : "not found");
            // telemetry.addData("CRServo (gb_crservo)", gb_hasCRServo ? "FOUND" : "not found"); // commented out
        } catch (Exception e) {
            telemetry.addData("Error", "Hardware init failed: " + e.getMessage());
        }
        telemetry.update();
    }

    private void displayMotorSpecifications() {
        telemetry.addLine("=== Motor Specifications ===");
        telemetry.addData("Motor1", "Intake (90% fixed power)");
        telemetry.addData("Motor2", "Shooting 1 (variable power)");
        telemetry.addData("Motor3", "Shooting 2 (variable power)");
        telemetry.addData("Motor4", "cb_dcMotor (variable power)");
        telemetry.addData("Model", "goBilda 5203-2402-0019");
        telemetry.addData("Bare Motor Speed", "%.0f RPM @ 12V", BARE_MOTOR_RPM);
        telemetry.addData("Gear Ratio", "%.1f:1", GEAR_RATIO);
        telemetry.addData("Output Shaft Speed", "%.0f RPM @ 12V (100%%)", FREE_RUN_RPM);
        telemetry.addData("Nominal Voltage", "%.1fV", NOMINAL_VOLTAGE);
        telemetry.addLine("");
    }

    private void displayControls() {
        telemetry.addLine("=== INTAKE CONTROLS (Motor1) ===");
        telemetry.addData("A Button", "Toggle intake ON/OFF");
        telemetry.addData("DPAD Left/Right", "Change direction");
        telemetry.addLine("");
        telemetry.addLine("=== SHOOTING CONTROLS (Motor2/3/4) ===");
        telemetry.addData("B Button", "Toggle shooting + cb_dcMotor ON/OFF");
        telemetry.addData("DPAD Up/Down", "Increase/Decrease power");
        telemetry.addData("X Button", "Set 75% power");
        telemetry.addData("Y Button", "Set 100% power");
        telemetry.addData("Right Bumper", "Fine control (2.5%)");
        telemetry.addData("Left Bumper", "Normal control (5%)");
        telemetry.addLine("");
        telemetry.addLine("=== goBILDA SERVO CONTROLS ===");
        telemetry.addData("Positional (servo1)", "RT+=pos, LT-=pos | LS button=min | RS button=max");
        // telemetry.addData("CR (gb_crservo)", "Power = RT - LT"); // commented out
        telemetry.addLine("");
    }

    private void checkIntakeControls() {
        // Get current button states for intake
        boolean currentA = gamepad1.a;
        boolean currentLeft = gamepad1.dpad_left;
        boolean currentRight = gamepad1.dpad_right;

        // Toggle intake motor on button press
        if (currentA && !im_lastAState) {
            im_intakeRunning = !im_intakeRunning;
        }

        // Change intake direction
        if (currentLeft && !im_lastLeftState) {
            im_intakeForward = false; // Reverse
        }
        if (currentRight && !im_lastRightState) {
            im_intakeForward = true; // Forward
        }

        // Update last button states
        im_lastAState = currentA;
        im_lastLeftState = currentLeft;
        im_lastRightState = currentRight;
    }

    private void checkShootingControls() {
        // Get current button states for shooting
        boolean sm_currentB = gamepad1.b;
        boolean sm_currentUp = gamepad1.dpad_up;
        boolean sm_currentDown = gamepad1.dpad_down;
        boolean sm_currentX = gamepad1.x;
        boolean sm_currentY = gamepad1.y;
        boolean sm_rightBumper = gamepad1.right_bumper;
        boolean sm_leftBumper = gamepad1.left_bumper;

        // Toggle shooting motors and cb_dcMotor on button press
        if (sm_currentB && !sm_lastBState) {
            sm_shootingRunning = !sm_shootingRunning;
        }

        // Adjust power increment
        if (sm_rightBumper) {
            sm_shootingPowerIncrement = 0.025; // Fine control: 2.5%
        } else if (sm_leftBumper) {
            sm_shootingPowerIncrement = 0.05;  // Normal control: 5%
        }

        // Increase shooting power
        if (sm_currentUp && !sm_lastUpState) {
            sm_shootingPower = Math.min(1.0, sm_shootingPower + sm_shootingPowerIncrement);
        }

        // Decrease shooting power
        if (sm_currentDown && !sm_lastDownState) {
            sm_shootingPower = Math.max(0.0, sm_shootingPower - sm_shootingPowerIncrement);
        }

        // Preset power levels for shooting
        if (sm_currentX) {
            sm_shootingPower = 0.75;
        } else if (sm_currentY) {
            sm_shootingPower = 1.0;
        }

        // Update last button states
        sm_lastBState = sm_currentB;
        sm_lastUpState = sm_currentUp;
        sm_lastDownState = sm_currentDown;
    }

    private void checkServoControls() {
        double lt = gamepad1.left_trigger;   // 0..1
        double rt = gamepad1.right_trigger;  // 0..1

        // Positional servo control via triggers and stick buttons
        if (gb_hasServo) {
            if (rt > 0.05) {
                gb_servoPos = Math.min(0.17, gb_servoPos + GB_SERVO_STEP * rt);
            }
            if (lt > 0.05) {
                gb_servoPos = Math.max(0.0, gb_servoPos - GB_SERVO_STEP * lt);
            }
            if (gamepad1.left_stick_button) {
                gb_servoPos = 0.0; // fully retracted
            }
            if (gamepad1.right_stick_button) {
                gb_servoPos = 0.17; // fully extended
            }
        }
    }

    private void updateMotorTelemetry() {
        // Get current motor velocities
        double intakeSpeed = Math.abs(im_intakeMotor.getVelocity());
        double sm_shootingSpeed1 = Math.abs(sm_shootingMotor1.getVelocity());
        double sm_shootingSpeed2 = Math.abs(sm_shootingMotor2.getVelocity());
        double cb_speed = Math.abs(cb_dcMotor.getVelocity());

        // Convert to RPM
        double intakeRPM = (intakeSpeed * 60) / ENCODER_CPR;
        double sm_shootingRPM1 = (sm_shootingSpeed1 * 60) / ENCODER_CPR;
        double sm_shootingRPM2 = (sm_shootingSpeed2 * 60) / ENCODER_CPR;
        double cb_RPM = (cb_speed * 60) / ENCODER_CPR;

        // Update performance tracking
        updatePerformanceStats(intakeRPM, sm_shootingRPM1, sm_shootingRPM2, cb_RPM);

        // Calculate servo angle from position
        double gb_servoAngle = gb_servoPos * (GB_SERVO_MAX_ANGLE - GB_SERVO_MIN_ANGLE) + GB_SERVO_MIN_ANGLE;

        // Clear and update telemetry display
        telemetry.clear();

        // Header section
        telemetry.addLine("=== DUAL SHOOTING MOTOR + CB MOTOR + SERVO TEST ===");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addLine("");

        // Intake Motor Status (Motor1)
        telemetry.addLine("=== INTAKE MOTOR (Motor1) ===");
        telemetry.addData("State", im_intakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Direction", im_intakeForward ? "FORWARD (Intake)" : "REVERSE (Eject)");
        telemetry.addData("Power Level", "%.1f%% (Fixed)", im_intakePower * 100);
        telemetry.addData("Current RPM", "%.1f RPM", intakeRPM);
        telemetry.addData("Expected RPM @ Power", "%.1f RPM", FREE_RUN_RPM * im_intakePower);
        telemetry.addData("Max Possible RPM", "%.0f RPM @ 100%%", FREE_RUN_RPM);
        telemetry.addLine("");

        // Shooting Motor 1 Status (Motor2)
        telemetry.addLine("=== SHOOTING MOTOR 1 (Motor2) ===");
        telemetry.addData("State", sm_shootingRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Power Level", "%.1f%% (Variable)", sm_shootingPower * 100);
        telemetry.addData("Control Mode", sm_shootingPowerIncrement == 0.025 ? "FINE (2.5%)" : "NORMAL (5%)");
        telemetry.addData("Current RPM", "%.1f RPM", sm_shootingRPM1);
        telemetry.addData("Expected RPM @ Power", "%.1f RPM", FREE_RUN_RPM * sm_shootingPower);
        telemetry.addData("Max Possible RPM", "%.0f RPM @ 100%%", FREE_RUN_RPM);
        telemetry.addLine("");

        // Shooting Motor 2 Status (Motor3)
        telemetry.addLine("=== SHOOTING MOTOR 2 (Motor3) ===");
        telemetry.addData("State", sm_shootingRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Power Level", "%.1f%% (Variable)", sm_shootingPower * 100);
        telemetry.addData("Control Mode", sm_shootingPowerIncrement == 0.025 ? "FINE (2.5%)" : "NORMAL (5%)");
        telemetry.addData("Current RPM", "%.1f RPM", sm_shootingRPM2);
        telemetry.addData("Expected RPM @ Power", "%.1f RPM", FREE_RUN_RPM * sm_shootingPower);
        telemetry.addData("Max Possible RPM", "%.0f RPM @ 100%%", FREE_RUN_RPM);
        telemetry.addLine("");

        // cb_dcMotor Status (Motor4)
        telemetry.addLine("=== CB DC MOTOR (Motor4) ===");
        telemetry.addData("State", sm_shootingRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Power Level", "%.1f%% (Variable)", sm_shootingPower * 100);
        telemetry.addData("Control Mode", sm_shootingPowerIncrement == 0.025 ? "FINE (2.5%)" : "NORMAL (5%)");
        telemetry.addData("Current RPM", "%.1f RPM", cb_RPM);
        telemetry.addData("Expected RPM @ Power", "%.1f RPM", FREE_RUN_RPM * sm_shootingPower);
        telemetry.addData("Max Possible RPM", "%.0f RPM @ 100%%", FREE_RUN_RPM);
        telemetry.addLine("");

        // goBILDA Servo status
        telemetry.addLine("=== goBILDA SERVO ===");
        telemetry.addData("Positional servo present", gb_hasServo);
        if (gb_hasServo) {
            telemetry.addData("Position", "%.2f", gb_servoPos);
            telemetry.addData("Angle (deg)", "%.1f", gb_servoAngle);
        }
        // telemetry.addData("CR servo present", gb_hasCRServo);
        // if (gb_hasCRServo) telemetry.addData("CR power (RT-LT)", "%.2f", gamepad1.right_trigger - gamepad1.left_trigger); // commented out
        telemetry.addLine("");

        // Performance statistics
        telemetry.addLine("=== STATISTICS ===");
        telemetry.addData("Max Intake RPM", "%.1f RPM", im_maxIntakeRPM);
        telemetry.addData("Max Shooting RPM 1", "%.1f RPM", sm_maxShootingRPM1);
        telemetry.addData("Max Shooting RPM 2", "%.1f RPM", sm_maxShootingRPM2);
        telemetry.addData("Max CB Motor RPM", "%.1f RPM", cb_maxRPM);
        telemetry.addData("Avg Intake RPM", "%.1f RPM", im_avgIntakeRPM);
        telemetry.addData("Avg Shooting RPM 1", "%.1f RPM", sm_avgShootingRPM1);
        telemetry.addData("Avg Shooting RPM 2", "%.1f RPM", sm_avgShootingRPM2);
        telemetry.addData("Avg CB Motor RPM", "%.1f RPM", cb_avgRPM);
        telemetry.addLine("");

        // System status
        if (im_intakeRunning && sm_shootingRunning) {
            telemetry.addData("System", "ALL DC MOTORS RUNNING");
        } else if (im_intakeRunning || sm_shootingRunning) {
            telemetry.addData("System", "One or more DC motors running");
        } else {
            telemetry.addData("System", "All DC motors stopped");
        }

        // Controls reminder
        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Intake", "A=Toggle | L/R=Direction");
        telemetry.addData("Shooting+CB", "B=Toggle | Up/Down=Power | X=75% | Y=100%");
        telemetry.addData("Servo", "RT+=pos / LT-=pos | LS=min | RS=max");
        // telemetry.addData("CR", "power=RT-LT"); // commented out

        telemetry.update();
    }

    private void updatePerformanceStats(double intakeRPM, double sm_shootingRPM1, double sm_shootingRPM2, double cb_RPM) {
        if (im_intakeRunning && intakeRPM > 0) {
            im_maxIntakeRPM = Math.max(im_maxIntakeRPM, intakeRPM);
            if (im_intakeSampleCount == 0) {
                im_avgIntakeRPM = intakeRPM;
            } else {
                im_avgIntakeRPM = (im_avgIntakeRPM * im_intakeSampleCount + intakeRPM) / (im_intakeSampleCount + 1);
            }
            im_intakeSampleCount++;
        }

        if (sm_shootingRunning && sm_shootingRPM1 > 0) {
            sm_maxShootingRPM1 = Math.max(sm_maxShootingRPM1, sm_shootingRPM1);
            if (sm_shootingSampleCount1 == 0) {
                sm_avgShootingRPM1 = sm_shootingRPM1;
            } else {
                sm_avgShootingRPM1 = (sm_avgShootingRPM1 * sm_shootingSampleCount1 + sm_shootingRPM1) / (sm_shootingSampleCount1 + 1);
            }
            sm_shootingSampleCount1++;
        }

        if (sm_shootingRunning && sm_shootingRPM2 > 0) {
            sm_maxShootingRPM2 = Math.max(sm_maxShootingRPM2, sm_shootingRPM2);
            if (sm_shootingSampleCount2 == 0) {
                sm_avgShootingRPM2 = sm_shootingRPM2;
            } else {
                sm_avgShootingRPM2 = (sm_avgShootingRPM2 * sm_shootingSampleCount2 + sm_shootingRPM2) / (sm_shootingSampleCount2 + 1);
            }
            sm_shootingSampleCount2++;
        }

        if (sm_shootingRunning && cb_RPM > 0) {
            cb_maxRPM = Math.max(cb_maxRPM, cb_RPM);
            if (cb_sampleCount == 0) {
                cb_avgRPM = cb_RPM;
            } else {
                cb_avgRPM = (cb_avgRPM * cb_sampleCount + cb_RPM) / (cb_sampleCount + 1);
            }
            cb_sampleCount++;
        }
    }
}