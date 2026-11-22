/* This is updated code as of 10/19/2025 */
package org.firstinspires.ftc.teamcode.teleop;

// These are names we made for each wheel motor in another file (ChassisConstants)
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_REAR_MOTOR_NAME;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// These are the “subsystems” — little robot parts that do special jobs
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingDC;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight3ASubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.teamcode.subsystems.UpDownSystem;

// These come from the NextFTC library
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Robot Centric TeleOp")
public class RobotCentricTeleOp extends NextFTCOpMode {

    // Alliance selection is stored during INIT for future alliance-specific behavior.
    private enum Alliance {
        BLUE,
        RED
    }

    // Default to blue unless the driver chooses red before pressing START.
    private Alliance selectedAlliance = Alliance.BLUE;

    // These are the robot's different systems (we’ll use them later)
    private ShootingSystem shootingSystem;
    //private ShootingDirectionServo shootingDirectionServo;
    private BallLoadingDC ballLoadingDC;
    private Intake intakeSystem;
    private Limelight3ASubsystem limelight3ASubsystem;
    private Light light;
    // Encoder-controlled mechanism operated by gamepad 1 after START.
    private UpDownSystem upDownSystem;
    // A neutral D-pad after START or movement is required before accepting a new press.
    private boolean upDownButtonsReady = false;
    private PinpointLocalizer localizer;
    private Follower follower;
    private Command driverControlled;
    // Only the follower controls the drivetrain while automatic movement is active.
    private boolean automaticMovement = false;
    private Pose turnTarget;
    private long turnStartedNanos;
    private static final double TURN_TIMEOUT_SECONDS = 3.0;

    // Remember joystick motor directions before switching to Pedro's directions.
    // A later method will use these values to restore joystick driving.
    private DcMotorSimple.Direction savedFrontLeftDirection;
    private DcMotorSimple.Direction savedFrontRightDirection;
    private DcMotorSimple.Direction savedBackLeftDirection;
    private DcMotorSimple.Direction savedBackRightDirection;
    protected final IMUEx imu = new IMUEx("imu", Direction.DOWN, Direction.FORWARD).zeroed();

    // This is the "constructor" — runs once when the program starts loading
    public RobotCentricTeleOp() {

        // We connect our subsystems to the robot and give them access to telemetry (data shown on driver station)
        shootingSystem = ShootingSystem.getInstance(telemetry);
        //shootingDirectionServo = ShootingDirectionServo.getInstance(telemetry);
        ballLoadingDC = BallLoadingDC.getInstance(telemetry);
    }

    @Override
    public void onInit() {
        super.onInit();

        // Obtain Intake after the SDK has supplied the current OpMode's telemetry.
        intakeSystem = Intake.getInstance(telemetry);

        limelight3ASubsystem = Limelight3ASubsystem.getInstance(this.hardwareMap, telemetry, imu);

        // Record the upper reference during INIT; start with the mechanism raised.
        upDownSystem = UpDownSystem.getInstance(telemetry);
        upDownSystem.initialize();

        // Obtain the light after the SDK has supplied this OpMode's telemetry.
        light = Light.getInstance(telemetry);

        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(PoseFactory.degrees().of(0, 0, 0));
        initializeFollower();
    

        // Here we "add" all these subsystems so NextFTC can manage and update them automatically
        addComponents(
                new SubsystemComponent(limelight3ASubsystem),
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(intakeSystem),
                new SubsystemComponent(ballLoadingDC),
                new SubsystemComponent(light),
                new SubsystemComponent(upDownSystem), // Updates mechanism state telemetry.
                BulkReadComponent.INSTANCE,   // reads all sensors at once for faster updates
                BindingsComponent.INSTANCE    // helps connect buttons on the gamepads to commands
        );

        // Apply the default alliance color; Light.periodic() displays it during INIT.
        updateAllianceLight();
    }

    private void initializeFollower() {
        // Mecanum's constructor changes motor settings. Save them for joystick driving.
        DcMotorEx[] motors = {frontLeftMotor.getMotor(), frontRightMotor.getMotor(),
                backLeftMotor.getMotor(), backRightMotor.getMotor()};
        DcMotorSimple.Direction[] directions = new DcMotorSimple.Direction[4];
        DcMotor.ZeroPowerBehavior[] brakeModes = new DcMotor.ZeroPowerBehavior[4];
        for (int i = 0; i < motors.length; i++) {
            directions[i] = motors[i].getDirection();
            brakeModes[i] = motors[i].getZeroPowerBehavior();
        }

        try {
            follower = new Follower(localizer,
                    new Mecanum(hardwareMap, Constants.drivetrainConfig),
                    new Foresight(Constants.foresightConfig));
        } finally {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setDirection(directions[i]);
                motors[i].setZeroPowerBehavior(brakeModes[i]);
            }
        }
    }

    @Override
    public void onUpdate() {
        // One handler owns both mechanism buttons to prevent conflicting target requests.
        handleUpDownButtons();
        if (automaticMovement) {
            if (turnTarget != null) {
                // Turning uses TeleOp directions and controls heading only.
                updateTurn();
            } else {
                // Other automatic movement still uses Pedro and its motor directions.
                follower.update();
                if (!follower.following()) {
                    automaticMovement = false;
                    resumeDriverControl();
                }
            }
        } else {
            // Joysticks control driving; only update the measured position.
            localizer.update();
        }
        Pose pose = localizer.state().pose();

        telemetry.addData("<=====Robot Location=====>", "");
        telemetry.addData("Robot Pose", "X: %.2f in | Y: %.2f in | Heading: %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    // These are our 4 drive motors (the robot’s wheels)
    // The MotorEx class is like a smarter motor object from the NextFTC library
    private final MotorEx frontLeftMotor = new MotorEx(LEFT_FRONT_MOTOR_NAME);
    private final MotorEx frontRightMotor = new MotorEx(RIGHT_FRONT_MOTOR_NAME).reversed(); // reversed so both sides move forward correctly
    private final MotorEx backLeftMotor = new MotorEx(LEFT_REAR_MOTOR_NAME);
    private final MotorEx backRightMotor = new MotorEx(RIGHT_REAR_MOTOR_NAME).reversed();

    @Override
    public void onWaitForStart() {
        Alliance previousAlliance = selectedAlliance;
        // Gamepad 1 selects the alliance only during INIT; no movement is started.
        if (gamepad1.dpad_up) {
            selectedAlliance = Alliance.BLUE;
        } else if (gamepad1.dpad_down) {
            selectedAlliance = Alliance.RED;
        }

        // Update only when the driver changes alliance, not every loop while held.
        if (selectedAlliance != previousAlliance) {
            updateAllianceLight();
        }

        // Show the selection so the driver can confirm it before pressing START.
        telemetry.addData("Select alliance", "D-pad up: BLUE | down: RED");
        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.update();
    }

    // This method runs when the driver presses the START button on the Driver Station
    @Override
    public void onStartButtonPressed() {

        // This tells the robot how to drive using mecanum wheels (which move in all directions)
        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),  // forward/backward
                () -> -(double) gamepad1.left_stick_x,  // Invert strafe so left stick left moves left.
                () -> -(double) gamepad1.right_stick_x  // Invert turn so right stick left turns left.
                //Gamepads.gamepad1().rightStickY(),  // forward/backward
                //Gamepads.gamepad1().rightStickX(),  // strafe left/right
                //Gamepads.gamepad1().leftStickX()  // turn left/right
        );

        // "Schedule" means start running that drive command
        driverControlled.schedule();

        // Require release after START so an INIT alliance selection cannot move the mechanism.
        // onUpdate() handles both buttons together instead of independent bindings.
        upDownButtonsReady = false;

        // Each B press requests a half-turn from the current heading, without X/Y correction.
        /*Gamepads.gamepad1().b().whenBecomesTrue(new InstantCommand(() -> {
            if (automaticMovement) return;

            Pose current = localizer.state().pose();
            // Capture a new relative target each time; updateTurn() uses only its heading.
            turnTarget = PoseFactory.degrees().of(current.x(), current.y(), Math.toDegrees(current.heading()) + 180);
            pauseDriverControl();
            // Keep TeleOp directions; onUpdate() will run rotation-only control.
            turnStartedNanos = System.nanoTime();
            automaticMovement = true;
        }));*/

        /* 🎮 GAMEPAD 2 CONTROLS (the second controller) */

        //X button → turn shooter on/off
        //Gamepads.gamepad1().x().whenBecomesTrue(shootingSystem.startStop);
        Gamepads.gamepad2().x().whenBecomesTrue(
            new SequentialGroup(
                    shootingSystem.startStop,
                    //new Delay(4.0), //For shooting motor to get to the right speed
                    ballLoadingDC.startStop
            )
        );

        // Y button → aim the shooter down
        //Gamepads.gamepad2().y().whenBecomesTrue(shootingDirectionServo.downShootingServo);
        Gamepads.gamepad2().y().whenBecomesTrue(
                new SequentialGroup(
                        ballLoadingDC.startStopReverse,
                        intakeSystem.startStopReverse
                )
        );

        // Left bumper → lower shooter power
        //Gamepads.gamepad2().leftBumper().whenBecomesTrue(shootingSystem.decreaseShootingPower);

        // Right bumper → increase shooter power
        //Gamepads.gamepad2().rightBumper().whenBecomesTrue(shootingSystem.increaseShootingPower);

        // Intake System Controls on Gamepad 2 as before
        //Gamepads.gamepad2().b().whenBecomesTrue(intakeSystem.reverse);
        Gamepads.gamepad2().a().whenBecomesTrue(intakeSystem.startStop);

/*
        // B button → reverse the intake (spit out the balls)
        //Gamepads.gamepad2().b().whenBecomesTrue(intakeSystem.reverse);

        // A button → aim the shooter up
        //Gamepads.gamepad2().a().whenBecomesTrue(shootingDirectionServo.upShootingServo);
*/
        // Back button → stop everything in the shooting system
        //Gamepads.gamepad2().back().whenBecomesTrue(shootingSystem.stopAllSubsystems);

        // D-Pad Left → run the ball loading servo backward (to load balls)
        //Gamepads.gamepad1().dpadLeft().whenBecomesTrue(BallLoadingDC.getInstance().reverse);

        // D-Pad Right → stop the ball loading servo
        //Gamepads.gamepad2().dpadRight().whenBecomesTrue(BallLoadingDC.getInstance().stopLoading);

        // D-Pad Left → run the ball loading servo backward (to load balls)
        //Gamepads.gamepad2().dpadUp().whenBecomesTrue(light.setPattern(Light.SOLID_BLUE_POSITION));
        /*Gamepads.gamepad2().dpadUp().whenBecomesTrue(
                new SequentialGroup(
                        light.setPattern(0.05),
                        new InstantCommand(() -> {
                            limelight3ASubsystem.switchLinelinePipeline(9);
                        }).requires(limelight3ASubsystem)
                )
        );*/

        // D-Pad Right → stop the ball loading servo
        //Gamepads.gamepad2().dpadDown().whenBecomesTrue(light.setPattern(Light.SOLID_RED_POSITION));
        /*Gamepads.gamepad2().dpadDown().whenBecomesTrue(
                new SequentialGroup(
                        light.setPattern(-0.05),
                        new InstantCommand(() -> {
                            limelight3ASubsystem.switchLinelinePipeline(8);
                        }).requires(limelight3ASubsystem)
                )
        );*/
    }

    /**
     * Accepts one Up or Down press only when idle and after both buttons were released.
     * Conflicting presses and presses during travel are discarded, never queued.
     */
    private void handleUpDownButtons() {
        boolean upPressed = gamepad1.dpad_up;
        boolean downPressed = gamepad1.dpad_down;

        // Do not reverse an active move; require another release after it finishes.
        if (upDownSystem.isMoving()) {
            upDownButtonsReady = false;
            return;
        }

        // Only a neutral D-pad while idle arms the next fresh press.
        if (!upPressed && !downPressed) {
            upDownButtonsReady = true;
            return;
        }

        // Reject both buttons together, including releasing just one afterward.
        if (upPressed && downPressed) {
            upDownButtonsReady = false;
            return;
        }

        // Holding a button or pressing during the lockout cannot start another move.
        if (!upDownButtonsReady) return;
        upDownButtonsReady = false;

        // Schedule one existing target command and consume this press immediately.
        if (upPressed) {
            upDownSystem.moveUp.schedule();
        } else {
            upDownSystem.moveDown.schedule();
        }
    }

    @Override
    public void onStop() {
        // Stop automatic control without restarting the joystick command.
        if (automaticMovement) {
            if (turnTarget != null) {
                // Rotation-only control writes directly to the wheel motors.
                setTurnPower(0);
            } else {
                follower.stop();
            }
            turnTarget = null;
            automaticMovement = false;
        }
        // Release active position-control power when the Driver Station stops TeleOp.
        if (upDownSystem != null) {
            upDownSystem.stop();
        }
    }

    /**
     * Pauses joystick driving and stops all four drive motors.
     * Call after onStartButtonPressed() has created the driver command.
     * This does not start automatic movement or resume joystick control.
     */
    private void pauseDriverControl() {
        // Prevent the joystick command from writing new motor powers.
        driverControlled.cancel();

        // Clear the last driving powers so the motors are commanded to stop.
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * Saves joystick motor directions and applies the directions Pedro expects.
     * Call once after pauseDriverControl(), before starting automatic movement.
     * Do not call again until joystick directions have been restored: a second
     * call would overwrite the saved joystick settings with Pedro's settings.
     * This only changes motor directions; it does not start the follower.
     */
    private void prepareForFollower() {
        // Save the current hardware directions for returning to joystick control.
        savedFrontLeftDirection = frontLeftMotor.getMotor().getDirection();
        savedFrontRightDirection = frontRightMotor.getMotor().getDirection();
        savedBackLeftDirection = backLeftMotor.getMotor().getDirection();
        savedBackRightDirection = backRightMotor.getMotor().getDirection();

        // Use the same directions as the Pedro drivetrain configuration.
        frontLeftMotor.getMotor().setDirection(
                Constants.drivetrainConfig.frontLeftDirection.get());
        frontRightMotor.getMotor().setDirection(
                Constants.drivetrainConfig.frontRightDirection.get());
        backLeftMotor.getMotor().setDirection(
                Constants.drivetrainConfig.backLeftDirection.get());
        backRightMotor.getMotor().setDirection(
                Constants.drivetrainConfig.backRightDirection.get());
    }

    /**
     * Stops automatic movement and returns driving control to the joysticks.
     * Call only after prepareForFollower() has saved the joystick directions.
     * The caller must also stop updating the follower when manual control resumes.
     */
    private void resumeDriverControl() {
        // Stop Pedro before restoring directions or allowing joystick commands.
        follower.stop();

        // Restore the hardware directions that were used for joystick driving.
        frontLeftMotor.getMotor().setDirection(savedFrontLeftDirection);
        frontRightMotor.getMotor().setDirection(savedFrontRightDirection);
        backLeftMotor.getMotor().setDirection(savedBackLeftDirection);
        backRightMotor.getMotor().setDirection(savedBackRightDirection);

        // Restart the existing joystick command; do not create a second one.
        driverControlled.schedule();
    }

    /**
     * Hands driving control from the joysticks to Pedro and starts a target path.
     * Call only during active TeleOp, after the driver command has been created.
     * Repeated requests are ignored while automatic movement is already active.
     * Completion handling will be added before this method is connected to a button.
     *
     * @param x target X in inches, in the same frame as the position telemetry
     * @param y target Y in inches, in the same frame as the position telemetry
     * @param headingDegrees target robot heading in degrees
     */
    private void startAutomaticMovement(double x, double y, double headingDegrees) {
        // Avoid saving Pedro's directions over the original joystick directions.
        if (automaticMovement) return;

        // Build the path before changing who controls the drive motors.
        Path path = createPathTo(x, y, headingDegrees);
        pauseDriverControl();
        prepareForFollower();

        // Set the path, then let onUpdate() run the follower each loop.
        follower.follow(path);
        automaticMovement = true;
    }

    /**
     * Calculates the remaining turn in radians using the latest measured heading.
     * Call only while turnTarget is set. This helper does not update localization
     * or command the motors; updateTurn() handles those operations.
     */
    private double getTurnHeadingError() {
        double difference = turnTarget.heading() - localizer.state().pose().heading();
        // Wrap the error to +/- pi so crossing 0/360 degrees uses the shorter turn.
        return Math.atan2(Math.sin(difference), Math.cos(difference));
    }

    /**
     * Applies opposite wheel powers to rotate without commanding translation.
     * Use with TeleOp motor directions, after pausing joystick control.
     * Passing zero stops all four wheels. Verify the heading direction on the robot.
     */
    private void setTurnPower(double power) {
        // Both wheels on each side receive the same power; the sides oppose each other.
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
    }

    /**
     * Updates a rotation-only turn using TeleOp motor directions.
     * Call with turnTarget set, joystick control paused, and turnStartedNanos recorded.
     * onUpdate() calls this while a rotation-only turn is active.
     */
    private void updateTurn() {
        // Read the latest pose before calculating the remaining angle.
        localizer.update();
        double headingError = getTurnHeadingError();

        // Finish based on heading only; this turn does not correct X or Y.
        boolean atTarget = Math.abs(headingError) <= Math.toRadians(2.0);
        boolean timedOut =
                (System.nanoTime() - turnStartedNanos) / 1e9 >= TURN_TIMEOUT_SECONDS;

        if (atTarget || timedOut) {
            // Stop the wheels before allowing the joystick command to run again.
            setTurnPower(0);
            turnTarget = null;
            automaticMovement = false;
            driverControlled.schedule();
            return;
        }

        // Reduce power near the target and limit it to 25% in either direction.
        double turnPower = Math.max(-0.25, Math.min(0.25, headingError * 0.5));
        setTurnPower(turnPower);
    }

    /**
     * Creates a straight path from the latest measured robot pose to a target.
     * Coordinates use the same reference frame as the position telemetry:
     * (0, 0, 0) is the pose set when TeleOp initializes.
     * This only builds a path; it does not start the follower or move the robot.
     *
     * @param x target X coordinate in inches
     * @param y target Y coordinate in inches
     * @param headingDegrees target robot heading in degrees
     * @return a straight path with heading interpolated from current to target
     */
    private Path createPathTo(double x, double y, double headingDegrees) {
        Pose currentPose = localizer.state().pose();
        Pose targetPose = PoseFactory.degrees().of(x, y, headingDegrees);

        // Travel along a straight line while changing heading along the path.
        return line(currentPose, targetPose).linear(currentPose, targetPose);
    }

    /** Sets the alliance color directly so selection works before START as well. */
    private void updateAllianceLight() {
        // Use absolute colors so repeating the selection does not change the pattern value.
        if (selectedAlliance == Alliance.BLUE) {
            light.selectColor(Light.SOLID_BLUE_POSITION);
        } else {
            light.selectColor(Light.SOLID_RED_POSITION);
        }
    }

}
