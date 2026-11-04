package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class BallLoadingDC implements Subsystem {

    private static final double SHOOTER_TOLERANCE_TPS = 50.0;
    private boolean loadingRequested = false;

    // We only want ONE Intake object in the whole program (Singleton pattern)
    private static BallLoadingDC INSTANCE = null;

    // Used to show messages and data on the Driver Station phone
    private Telemetry telemetry;

    /**
     * This sets up the intake system with telemetry.
     * It’s private so only this class can make it.
     */
    private BallLoadingDC(Telemetry telemetry) {

        this.telemetry = telemetry;
    }

    /**
     * Gets or creates the one Intake system for the robot.
     * If it doesn’t exist yet, it makes a new one.
     */
    public static BallLoadingDC getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new BallLoadingDC(telemetry);
        }
        return INSTANCE;
    }

    /**
     * Returns the existing Intake instance (after it's created).
     * Throws an error if someone forgot to set it up first.
     */
    public static BallLoadingDC getInstance() {
        if (INSTANCE == null) {
            throw new NullPointerException("BallLoadingDC subsystem is not initialized yet");
        }
        return INSTANCE;
    }

    // This is the motor that makes the intake spin
    private MotorEx motor = new MotorEx("BallLoadMotor").reversed();

    // Power settings for different actions
    private static final double REVERSE_POWER = -0.75;  // Spin backward (push things OUT)
    private static final double STOP_POWER = 0.0;      // Stop spinning
    private static final double INTAKE_POWER = 0.75;    // Spin forward (pull things IN)

    private void updateLoading() {
        // Leave an active reverse command alone.
        if (motor.getPower() < 0.0) {
            loadingRequested = false;
            return;
        }

        if (loadingRequested) {
            boolean shooterAtSpeed = ShootingSystem.getInstance()
                    .isAtTargetVelocity(SHOOTER_TOLERANCE_TPS);

            if (shooterAtSpeed) {
                motor.setPower(INTAKE_POWER);
            } else {
                motor.setPower(STOP_POWER);
            }
        }
    }

    private void start() {
        loadingRequested = true;
        motor.setPower(STOP_POWER);
        updateLoading();
    }

    public Command startLoading = new InstantCommand(this::start)
            .requires(this);

    /**
     * Command to start or stop the intake motor.
     * When the button is pressed:
     * - If it's stopped, it starts spinning forward (to pull game pieces in)
     * - If it's already spinning, it stops
     */
    public Command startStop = new InstantCommand(() -> {
        if (loadingRequested || motor.getPower() != STOP_POWER) {
            stop();
        } else {
            start();
        }
    }).requires(this);

    /**
     * Stops the intake completely (used in emergencies or resets)
     */
    public void stop() {
        loadingRequested = false;
        motor.setPower(STOP_POWER);
    }

    /**
     * Command to reverse the intake motor.
     * This is used to push out game pieces that might be stuck.
     */
    public Command reverse = new InstantCommand(() -> motor.setPower(REVERSE_POWER))
            .requires(this);

    /** Toggles reverse on/off, cancelling any forward loading request. */
    public Command startStopReverse = new InstantCommand(() -> {
        if (motor.getPower() < 0.0) {
            stop();
        } else {
            loadingRequested = false;
            motor.setPower(REVERSE_POWER);
        }
    }).requires(this);

    public Command stopLoading = new InstantCommand(this::stop)
            .requires(this);

    /**
     * This method runs all the time during TeleOp.
     * It shows live data about what the intake motor is doing:
     *  - Power level (how hard it's spinning)
     *  - Direction (FORWARD or REVERSE)
     */
    @Override
    public void periodic() {
        updateLoading();
        telemetry.addData("<===== Ball Loading DC Subsystem =====>", "");
        telemetry.addData("Ball Loading ", "Power:%.2f", motor.getPower());
        //telemetry.update(); // optional, updates the screen immediately
    }

}
