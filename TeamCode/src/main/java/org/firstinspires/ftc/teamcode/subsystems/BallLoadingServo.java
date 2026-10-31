package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;

/**
 * This class controls the **ball-loading servo** on the robot.
 * It’s a *Continuous Rotation Servo*, which means it can spin
 * like a motor — not just move to a position.
 *
 * Think of it like a little conveyor belt motor
 * that helps push balls into the shooter.
 */
public class BallLoadingServo implements Subsystem {

    // We only want one copy of this class (Singleton pattern)
    private static BallLoadingServo INSTANCE = null;

    // The servo that spins to load the balls
    private CRServoEx crServo;

    // The name of the servo as set in the Control Hub configuration
    private static final String SERVO_NM = "ld_servo";

    // Keeps track of how fast the servo is spinning (-1 = backward, 1 = forward, 0 = stopped)
    private double currentPower = 0.0;

    // Used to show messages on the Driver Station phone
    private Telemetry telemetry;

    /**
     * This sets up the servo and telemetry system.
     * It’s private so that only this class can make it.
     */
    private BallLoadingServo(Telemetry telemetry) {
        this.telemetry = telemetry;
        crServo = new CRServoEx(SERVO_NM); // Connects to the real servo
        //crServo.setPower(0.0); // Start stopped (optional)
    }

    /**
     * Gives back the one and only BallLoadingServo.
     * If it doesn’t exist yet, it creates it.
     */
    public static BallLoadingServo getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new BallLoadingServo(telemetry);
        }
        return INSTANCE;
    }

    /**
     * Gives the existing servo controller (after it’s already created).
     * You must call the other getInstance() first.
     */
    public static BallLoadingServo getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("BallLoadingServo must be set up first!");
        }
        return INSTANCE;
    }

    /**
     * Changes how fast and in what direction the servo spins.
     * -1.0 = full backward, 1.0 = full forward, 0.0 = stop
     */
    public Command setContinuousPower(double power) {
        return new InstantCommand(() -> {
            if (crServo != null) {
                // Make sure power stays between -1 and 1
                currentPower = Math.max(-1.0, Math.min(1.0, power));
                crServo.setPower(currentPower);
            }
        }).requires(this);
    }

    /**
     * Spins the servo forward (positive direction) at a chosen speed.
     */
    public Command runForward(double power) {
        return setContinuousPower(Math.abs(power));
    }

    /**
     * Spins the servo forward at full speed.
     */
    public Command runForward() {
        return runForward(1.0);
    }

    /**
     * Spins the servo backward (negative direction) at a chosen speed.
     */
    public Command runBackward(double power) {
        return setContinuousPower(-Math.abs(power));
    }

    /**
     * Spins the servo backward at full speed.
     */
    public Command runBackward() {
        return runBackward(1.0);
    }

    /**
     * Stops the servo completely (no spinning).
     */
    public Command stopContinuous() {
        return setContinuousPower(0.0);
    }

    /**
     * Quick stop command that stops it immediately.
     */
    public void stop() {
        crServo.setPower(0.0);
    }

    /**
     * Runs repeatedly during TeleOp.
     * It shows what the servo is doing (forward, backward, or stopped)
     * and how much power it’s using.
     */
    @Override
    public void periodic() {
        telemetry.addData("<===== Ball Loading Servo =====>", "");
        telemetry.addData("CR Power", "%.3f", currentPower);

        // Show a friendly word for the direction
        String status = currentPower > 0 ? "Forward" :
                currentPower < 0 ? "Backward" : "Stopped";

        telemetry.addData("CR Status", status);
    }
}
