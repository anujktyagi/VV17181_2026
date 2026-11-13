package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/** Controls the shooter using encoder velocity in ticks per second. */
public class ShootingSystem implements Subsystem {
    private static ShootingSystem INSTANCE;

    // Starting value only; tune the shooting speed on the robot.
    private static final double DEFAULT_VELOCITY_TPS = 1130.0;
    private static final double VELOCITY_ADJUST_TPS = 15.0;

    private Telemetry telemetry;
    private final MotorEx shootingMotor1 = new MotorEx("sm1");
    private boolean initialized;
    private boolean running;
    private double targetVelocity = DEFAULT_VELOCITY_TPS;
    double P = 94.68;
    double F = 16.00;

    private ShootingSystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static ShootingSystem getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new ShootingSystem(telemetry);
        } else {
            // A singleton can survive between OpModes; refresh session state.
            INSTANCE.telemetry = telemetry;
            INSTANCE.initialized = false;
            INSTANCE.running = false;
            INSTANCE.targetVelocity = DEFAULT_VELOCITY_TPS;
        }
        return INSTANCE;
    }

    public static ShootingSystem getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("ShootingSystem must be initialized with telemetry first!");
        }
        return INSTANCE;
    }

    @Override
    public void initialize() {
        ensureInitialized();
    }

    private void ensureInitialized() {
        if (initialized) return;

        DcMotorEx motor = shootingMotor1.getMotor();
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients originalPIDF = new PIDFCoefficients(P, 0, 0, F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, originalPIDF);
        motor.setVelocity(0.0);
        running = false;
        initialized = true;
    }

    private void start() {
        ensureInitialized();
        shootingMotor1.getMotor().setVelocity(targetVelocity);
        running = true;
    }

    /** Stops the motor while retaining the selected target velocity. */
    public void stop() {
        ensureInitialized();
        shootingMotor1.getMotor().setVelocity(0.0);
        shootingMotor1.getMotor().setPower(0.0);
        running = false;
    }

    private void adjustVelocity(double delta) {
        ensureInitialized();
        targetVelocity = Math.max(0.0, targetVelocity + delta);
        if (running) {
            shootingMotor1.getMotor().setVelocity(targetVelocity);
        }
    }

    /** Checks the current speed against the target; tolerance is in ticks per second. */
    public boolean isAtTargetVelocity(double tolerance) {
        if (!Double.isFinite(tolerance) || tolerance < 0.0) {
            throw new IllegalArgumentException("Velocity tolerance must be finite and nonnegative");
        }
        ensureInitialized();
        return running && targetVelocity > 0.0
                && Math.abs(shootingMotor1.getMotor().getVelocity() - targetVelocity) <= tolerance;
    }

    public Command startShooting = new InstantCommand(this::start).requires(this);

    public Command startStop = new InstantCommand(() -> {
        if (running) stop();
        else start();
    }).requires(this);

    // Keep the existing names so TeleOp bindings continue to work.
    // Adjusting while stopped changes the next target without starting the motor.
    public Command increaseShootingPower = new InstantCommand(() ->
            adjustVelocity(VELOCITY_ADJUST_TPS)).requires(this);

    public Command decreaseShootingPower = new InstantCommand(() ->
            adjustVelocity(-VELOCITY_ADJUST_TPS)).requires(this);

    public Command stopAllSubsystems = new InstantCommand(() -> {
        stop();
        Intake.getInstance().stop();
    }).requires(this);

    @Override
    public void periodic() {
        // Also handles OpModes that register SubsystemComponent during onInit,
        // after NextFTC's component preInit phase has already run.
        ensureInitialized();
        double measuredVelocity = shootingMotor1.getMotor().getVelocity();
        telemetry.addData("<=====Shooting System=====>", "");
        telemetry.addData("Shooter running", running);
        telemetry.addData("Shooter selected target (ticks/s)", "%.1f", targetVelocity);
        telemetry.addData("Shooter measured (ticks/s)", "%.1f", measuredVelocity);
        //telemetry.update();
    }
}
