package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * Moves between two encoder positions, separated by 15% of a motor output revolution.
 * The mechanism must be physically lowered before its first initialization each OpMode.
 * Registered in RobotCentricTeleOp with D-pad up/down target commands.
 */
public class UpDownSystem implements Subsystem {
    // Share one subsystem instance, following the other mechanism subsystems.
    private static UpDownSystem INSTANCE;

    // Keep the current OpMode's telemetry reference for future status reporting.
    private Telemetry telemetry;

    // Lazy motor wrapper: match this name in the robot configuration later.
    // Declaring it does not initialize the hardware or command movement.
    private final MotorEx motor = new MotorEx("arm");
    // Reset for each OpMode so its motor setup runs once before use.
    private boolean initialized = false;

    // RUN_TO_POSITION uses positive power for both directions; the target chooses direction.
    private static final double MOVE_POWER = 0.75;
    private static final double TRAVEL_FRACTION = 0.45;
    // Change to -1 if decreasing encoder counts raise the mechanism.
    private static final int UP_DIRECTION = 1;
    private int lowerPosition;
    private int upperPosition;
    // True only while travelling to a target; reaching it removes motor power.
    private boolean movingToTarget = false;

    // Moves to the fixed upper target; repeated calls do not add more travel.
    public Command moveDown = new InstantCommand(() -> {
        ensureInitialized();
        moveToPosition(upperPosition);
    }).requires(this);

    // Returns to the lower position recorded during initialization.
    public Command moveUp = new InstantCommand(() -> {
        ensureInitialized();
        moveToPosition(lowerPosition);
    }).requires(this);

    /** Starts position control; periodic() removes power when the target is reached. */
    private void moveToPosition(int targetTicks) {
        DcMotorEx hardware = motor.getMotor();
        hardware.setTargetPosition(targetTicks);
        hardware.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.setPower(MOVE_POWER);
        movingToTarget = true;
    }

    /** Stops motor power; the configured BRAKE behavior remains enabled. */
    public void stop() {
        ensureInitialized();
        // Use the SDK motor consistently to avoid mixing cached power and position control.
        motor.getMotor().setPower(0.0);
        motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        movingToTarget = false;
    }

    // Stops movement when scheduled, for example on button release or in a sequence.
    // It is not connected to controls yet.
    public Command stopMoving = new InstantCommand(this::stop).requires(this);

    private UpDownSystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /** Creates the subsystem if needed and refreshes telemetry for the current OpMode. */
    public static UpDownSystem getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new UpDownSystem(telemetry);
        } else {
            INSTANCE.telemetry = telemetry;
        }
        // Prepare the singleton for a new OpMode; hardware access happens later.
        INSTANCE.initialized = false;
        return INSTANCE;
    }

    /** Performs one-time setup when NextFTC initializes this subsystem. */
    @Override
    public void initialize() {
        ensureInitialized();
    }

    /** Initializes hardware at zero power; does not raise or lower the mechanism. */
    private void ensureInitialized() {
        if (initialized) {
            return;
        }

        // Read resolution from the configured motor type; an encoder must be connected.
        DcMotorEx hardware = motor.getMotor();
        hardware.setPower(0.0);
        double ticksPerRev = hardware.getMotorType().getTicksPerRev();
        if (!Double.isFinite(ticksPerRev) || ticksPerRev <= 0.0) {
            throw new IllegalStateException("Configure a valid motor type for upDownMotor");
        }
        int travelTicks = (int) Math.round(ticksPerRev * TRAVEL_FRACTION);
        if (travelTicks < 1) {
            throw new IllegalStateException("Up/down travel must be at least one encoder tick");
        }
        // Record a reference, not an automatic homing move. Start physically lowered.
        lowerPosition = hardware.getCurrentPosition();
        upperPosition = lowerPosition + UP_DIRECTION * travelTicks;
        hardware.setTargetPosition(lowerPosition);
        hardware.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // BRAKE resists coasting but is not active position holding against gravity.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initialized = true;
        movingToTarget = false;
    }

    /** Reports encoder target, position, and requested movement; the OpMode sends telemetry. */
    @Override
    public void periodic() {
        // Initialize once before reading hardware, including when registration happens in onInit().
        ensureInitialized();
        DcMotorEx hardware = motor.getMotor();
        // Finish this move once the SDK reports arrival; do not keep correcting afterward.
        if (movingToTarget && !hardware.isBusy()) {
            stop();
        }
        double power = hardware.getPower();
        int currentPosition = hardware.getCurrentPosition();
        String state;

        // Power is positive in both directions, so use target error to label movement.
        if (power == 0.0) {
            state = "STOPPED";
        } else if (!hardware.isBusy()) {
            state = "AT TARGET";
        } else if ((hardware.getTargetPosition() - currentPosition) * UP_DIRECTION > 0) {
            state = "UP";
        } else {
            state = "DOWN";
        }

        telemetry.addData("<== Up/Down mechanism ==>", "State: %s | Power: %.2f", state, power);
        telemetry.addData("Up/Down ticks", "Current: %d | Target: %d | Lower: %d | Upper: %d",
                currentPosition, hardware.getTargetPosition(), lowerPosition, upperPosition);
        // Keep telemetry.update() in the OpMode so all subsystems display together.
    }

    /** Returns the subsystem after it has been created with getInstance(telemetry). */
    public static UpDownSystem getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("UpDownSystem must be created with telemetry first");
        }
        return INSTANCE;
    }
}
