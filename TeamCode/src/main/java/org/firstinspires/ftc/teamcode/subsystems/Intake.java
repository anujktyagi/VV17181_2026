package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * This class controls the **intake system** on the robot.
 *
 * The intake is the part that *picks up game pieces* (like rings or balls)
 * and pulls them into the robot using a spinning motor.
 */
public class Intake implements Subsystem {

    // We only want ONE Intake object in the whole program (Singleton pattern)
    private static Intake INSTANCE = null;

    // Used to show messages and data on the Driver Station phone
    private Telemetry telemetry;

    /**
     * This sets up the intake system with telemetry.
     * It’s private so only this class can make it.
     */
    private Intake(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Gets or creates the one Intake system for the robot.
     * If it doesn’t exist yet, it makes a new one.
     */
    public static Intake getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new Intake(telemetry);
        }
        return INSTANCE;
    }

    /**
     * Returns the existing Intake instance (after it's created).
     * Throws an error if someone forgot to set it up first.
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            throw new NullPointerException("Intake subsystem is not initialized yet");
        }
        return INSTANCE;
    }

    // This is the motor that makes the intake spin
    private MotorEx motor = new MotorEx("intakeMotor");

    // Power settings for different actions
    private static final double REVERSE_POWER = -1.0;  // Spin backward (push things OUT)
    private static final double STOP_POWER = 0.0;      // Stop spinning
    private static final double INTAKE_POWER = 1.0;    // Spin forward (pull things IN)

    public void startIntaking() {
        motor.setPower(INTAKE_POWER);
    }

    public Command startIntake = new InstantCommand(this::startIntaking)
            .requires(this);

    /**
     * Command to start or stop the intake motor.
     * When the button is pressed:
     * - If it's stopped, it starts spinning forward (to pull game pieces in)
     * - If it's already spinning, it stops
     */
    public Command startStop = new InstantCommand(() -> {
        if (motor.getPower() <= 0.10 && motor.getPower() >= 0.0)
            motor.setPower(INTAKE_POWER);   // Turn ON the intake
        else
            motor.setPower(STOP_POWER);     // Turn it OFF
    }).requires(this);

    /**
     * Stops the intake completely (used in emergencies or resets)
     */
    public void stop() {
        motor.setPower(STOP_POWER);
    }

    /**
     * Command to reverse the intake motor.
     * This is used to push out game pieces that might be stuck.
     */
    public Command reverse = new InstantCommand(() -> motor.setPower(REVERSE_POWER))
            .requires(this);

    /** Starts reverse if stopped or moving forward; stops if already reversing. */
    public Command startStopReverse = new InstantCommand(() -> {
        if (motor.getPower() < 0.0) {
            stop();
        } else {
            motor.setPower(REVERSE_POWER);
        }
    }).requires(this);

    /**
     * This method runs all the time during TeleOp.
     * It shows live data about what the intake motor is doing:
     *  - Power level (how hard it's spinning)
     *  - Direction (FORWARD or REVERSE)
     *  - Velocity (how fast it’s turning)
     */
    @Override
    public void periodic() {
        telemetry.addData("<===== Intake Subsystem =====>", "");
        telemetry.addData("Intake Power", "%.2f", motor.getPower());
        //telemetry.addData("Intake Direction", motor.getDirection());
        //telemetry.addData("Intake Velocity", "%.2f", motor.getVelocity());
        //telemetry.update(); // optional, updates the screen immediately
    }
}
