package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * This class controls the servo that changes where the robot shoots the ball.
 * Think of the servo like a tiny arm that can move up and down to aim.
 */
public class ShootingDirectionServo implements Subsystem {

    // We make only one copy of this class (Singleton pattern)
    private static ShootingDirectionServo INSTANCE = null;

    // The actual servo hardware on the robot
    private ServoEx servo;

    // This name must match what we set in the Control Hub configuration
    private static final String SERVO_NM = "sm_servo";

    // Used to show info on the Driver Station screen
    private Telemetry telemetry;

    // This constructor sets up the servo and telemetry system
    private ShootingDirectionServo(Telemetry telemetry) {
        this.telemetry = telemetry;
        servo = new ServoEx(SERVO_NM); // Connects to the servo using its name
    }

    /**
     * This method gives you the servo controller.
     * If it doesn’t exist yet, it creates it.
     */
    public static ShootingDirectionServo getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new ShootingDirectionServo(telemetry);
        }
        return INSTANCE;
    }

    /**
     * This method gives the already-created servo controller.
     * (You must call the one above first to set it up.)
     */
    public static ShootingDirectionServo getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("You must set up the servo first using telemetry!");
        }
        return INSTANCE;
    }

    // The limits of how far the servo can move
    private final double minPos = 1.0; // Highest position
    private final double maxPos = 0.8; // Lowest position

    // How much the servo moves each time you press a button
    private static final double SERVO_POS_INCREMENT = 0.05;

    /**
     * Command to move the shooting servo UP a little.
     * This makes the robot aim higher.
     */
    public Command upShootingServo = new InstantCommand(() -> {
        if (servo != null) {
            // Increases servo position but doesn’t go past its limit
            double clampedPosition = Math.min(minPos, servo.getPosition() + SERVO_POS_INCREMENT);
            servo.setPosition(clampedPosition);
        }
    }).requires(this);

    /**
     * Command to move the shooting servo DOWN a little.
     * This makes the robot aim lower.
     */
    public Command downShootingServo = new InstantCommand(() -> {
        if (servo != null) {
            // Decreases servo position but doesn’t go below its limit
            double clampedPosition = Math.max(maxPos, servo.getPosition() - SERVO_POS_INCREMENT);
            servo.setPosition(clampedPosition);
        }
    }).requires(this);

    /**
     * This runs over and over during the match.
     * It shows the servo’s position on the Driver Station screen.
     */
    @Override
    public void periodic() {
        telemetry.addData("<===== Shooting Direction Servo =====>", "");
        if (servo != null) {
            double currentPos = servo.getPosition(); // Find where the servo is
            String positionName = getCurrentPositionName(currentPos); // Give it a name (like "Min" or "Max")
            telemetry.addData("Current Position", positionName);
            telemetry.addData("Position Value", "%.3f", currentPos);
        } else {
            telemetry.addData("Servo Status", "Not Initialized");
        }
        //telemetry.update(); // Uncomment if you want live updates
    }

    /**
     * This helps figure out what position the servo is currently in.
     * It compares the servo’s number with the known positions.
     */
    private String getCurrentPositionName(double currentPos) {
        double tolerance = 0.01; // Small wiggle room for comparing numbers

        if (Math.abs(currentPos - minPos) < tolerance) {
            return "Min"; // Fully up
        } else if (Math.abs(currentPos - maxPos) < tolerance) {
            return "Max"; // Fully down
        } else {
            return "Custom"; // Somewhere in between
        }
    }
}
