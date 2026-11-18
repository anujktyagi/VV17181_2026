package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Light implements Subsystem {
    private static Light INSTANCE = null;

    // Telemetry for displaying the selected and commanded light positions.
    private Telemetry telemetry;
    private Light(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static Light getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new Light(telemetry);
        }
        // The singleton survives OpMode runs; always use this run's telemetry.
        INSTANCE.telemetry = telemetry;
        return INSTANCE;
    }

    // Servo positions for the goBILDA RGB Light (approximate common values)
    // Refer to this page for more insights on color -> value https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    // These specific values correspond to persistent, solid colors.
    public static final double OFF_POSITION = 0.1; // Initially it was .6
    public static final double SOLID_RED_POSITION = 0.28;
    public static final double SOLID_GREEN_POSITION = 0.5;
    public static final double SOLID_BLUE_POSITION = 0.65;
    public static final double SOLID_PURPLE_POSITION = 0.70;

    // Renamed from 'brightness' to 'targetPosition' to reflect the hardware
    private double targetPosition = OFF_POSITION; // Default to OFF or safe state

    private final ServoEx light = new ServoEx("Light");
    private Light() { }

    @Override
    public void periodic() {

        // Default to the selected alliance color.
        double currentPosition = targetPosition;

        // Temporarily show green when the shooting-position condition is met.
        if (Limelight3ASubsystem.getInstance().getIdentifiedShootingPosition() != '0') {
            currentPosition = SOLID_GREEN_POSITION;
        }

        // Send only the final color so each loop commands one position.
        light.setPosition(currentPosition);
        telemetry.addData("LED Light", "Alliance: %.2f | Commanded: %.2f",
                targetPosition, currentPosition);
    }

    @Override
    public void initialize() {
        // We will command the light to turn on in the TeleOp code instead of here
        // The periodic() loop will handle applying the default 'targetPosition'
    }

    /**
     * Creates a command that selects an absolute color/pattern position.
     * Repeating the same selection keeps the same target instead of adding to it.
     * periodic() applies the target, with the existing green override taking priority.
     */
    public Command setColor(double position) {
        return new InstantCommand(() -> selectColor(position)).requires(this);
    }

    /** Sets the target directly during INIT, when scheduled commands are not running yet. */
    public void selectColor(double position) {
        targetPosition = Math.max(0.10, Math.min(1.0, position));
    }

    // Adjusts the current target by the supplied amount; existing bindings use this behavior.
    public Command setPattern(double position) {
        return new InstantCommand(() -> {
            this.targetPosition = Math.max(0.10, Math.min(1.0, targetPosition + position));
        }).requires(this);
    }
}
