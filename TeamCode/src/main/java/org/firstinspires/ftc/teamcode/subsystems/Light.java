package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Light implements Subsystem {
    private static Light INSTANCE = null;

    // Telemetry for displaying motor status
    private Telemetry telemetry;
    private Light(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static Light getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new Light(telemetry);
        }
        return INSTANCE;
    }

    // Servo positions for the goBILDA RGB Light (approximate common values)
    // Refer to this page for more insights on color -> value https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    // These specific values correspond to persistent, solid colors.
    public static final double OFF_POSITION = 0.1; // Initially it was .6
    public static final double SOLID_RED_POSITION = 0.7;
    public static final double SOLID_GREEN_POSITION = 0.5;
    public static final double SOLID_BLUE_POSITION = 0.8;
    public static final double SOLID_PURPLE_POSITION = 0.70;

    // Renamed from 'brightness' to 'targetPosition' to reflect the hardware
    private double targetPosition = OFF_POSITION; // Default to OFF or safe state

    private final ServoEx light = new ServoEx("Light");
    private Light() { }

    @Override
    public void periodic() {

        // The periodic loop constantly applies the target position, ensuring the color/pattern stays active
        light.setPosition(targetPosition);
        // Change the light to GREEN when the robot position (a,b,x,y) is ideal for shooting. Zero (0) is default.
        if(Limelight3ASubsystem.getInstance().getIdentifiedShootingPosition() != '0') {
            light.setPosition(SOLID_GREEN_POSITION);
        }
        telemetry.addData("Led Light ", "Default: %.2f | Current: %.2f", targetPosition, light.getPosition());
    }

    @Override
    public void initialize() {
        // We will command the light to turn on in the TeleOp code instead of here
        // The periodic() loop will handle applying the default 'targetPosition'
    }

    // New method name: setPattern, which takes the specific servo position constant
    public Command setPattern(double position) {
        return new InstantCommand(() -> {
            this.targetPosition = Math.max(0.10, Math.min(1.0, targetPosition + position));
        }).requires(this);
    }
}
