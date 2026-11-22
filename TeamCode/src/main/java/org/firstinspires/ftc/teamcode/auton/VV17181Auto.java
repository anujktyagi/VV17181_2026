package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingDC;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight3ASubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@Autonomous
public class VV17181Auto extends NextFTCOpMode {

    // Names for autonomous routes from the straight and side starting positions.
    private enum AutoRoute {
        ROUTE_STRAIGHT,
        ROUTE_SIDE,
        // Placeholder name until the third route is tuned.
        ROUTE_SHORT
    }

    // Cycle through routes in the order declared in AutoRoute.
    private static final AutoRoute[] ROUTES = AutoRoute.values();

    // Default to the straight route until the driver selects another during INIT.
    private AutoRoute selectedRoute = AutoRoute.ROUTE_STRAIGHT;

    // Alliance choice is selected during INIT; it does not mirror route coordinates yet.
    private enum Alliance {
        BLUE,
        RED
    }

    // Default to blue unless the driver selects red before pressing START.
    private Alliance selectedAlliance = Alliance.BLUE;

    private static double START_DELAY_SECONDS = 0.0;
    private static double SHOOTING_RUN_SECONDS = 5.0;
    private static double PICKUP_TIMEOUT_SECONDS = 5.0;

    private Follower follower;
    private boolean pickupComplete;
    private ShootingSystem shootingSystem;
    private BallLoadingDC ballLoadingDC;
    private Intake intakeSystem;
    // Limelight uses the same IMU mounting configuration as RobotCentricTeleOp.
    private final IMUEx imu = new IMUEx("imu", Direction.DOWN, Direction.FORWARD).zeroed();
    private Limelight3ASubsystem limelight3ASubsystem;
    // Displays the alliance color, with the existing Limelight green override.
    private Light light;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Active poses are assigned from the selected route before they are used.
    private Pose startPose;
    private Pose ballPose;
    private Pose ballPickPose;
    private Pose parkAlignPose;
    private Pose parkPose;

    /** Assigns inches and degrees relative to the selected route's starting position. */
    private void applySelectedRoute() {
        START_DELAY_SECONDS = 0.0; // Only ROUTE_THIRD overrides this.
        // Separate cases allow each route's poses to be tuned independently.
        switch (selectedRoute) {
            case ROUTE_STRAIGHT:
                // Straight route: coordinates relative to its starting position.
                startPose = poseFactory.of(0, 0, 0);
                ballPose = poseFactory.of(-4, -32, 0);
                ballPickPose = poseFactory.of(-1, -32, 0);
                parkAlignPose = poseFactory.of(-6, -24, 45);
                parkPose = poseFactory.of(-57, -40, 90);
                break;
            case ROUTE_SIDE:
                // Side route: intentionally identical for now; adjust these values later.
                startPose = poseFactory.of(0, 0, 0);
                ballPose = poseFactory.of(2, -16, 27);
                ballPickPose = poseFactory.of(4, -14, 27);
                parkAlignPose = poseFactory.of(-14, -10, 60);
                parkPose = poseFactory.of(-54, -50, 112);
                break;
            case ROUTE_SHORT:
                // Copy the straight route for now; tune these poses independently later.
                startPose = poseFactory.of(0, 0, 0);
                ballPose = poseFactory.of(0, 0, 0);
                ballPickPose = poseFactory.of(0, 0, 0);
                parkAlignPose = poseFactory.of(-7, 0, 0);
                parkPose = poseFactory.of(0, 30, 270);
                START_DELAY_SECONDS = 20.0;
                break;
        }
    }

    @Override
    public void onInit() {
        // Initialize poses before the follower reads startPose.
        applySelectedRoute();
        Scheduler.reset(); //Anuj: reset ivy schedular. This dependency ahd to be included manually

        shootingSystem = ShootingSystem.getInstance(telemetry);
        ballLoadingDC = BallLoadingDC.getInstance(telemetry);
        intakeSystem = Intake.getInstance(telemetry);
        // Connect the camera to this OpMode's hardware, telemetry, and IMU during INIT.
        limelight3ASubsystem = Limelight3ASubsystem.getInstance(hardwareMap, telemetry, imu);
        // Use this run's telemetry and reset the light to the default alliance color.
        light = Light.getInstance(telemetry);
        updateAllianceLight();

        addComponents(
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(ballLoadingDC),
                new SubsystemComponent(intakeSystem),
                // Run camera processing and add its telemetry during INIT and autonomous.
                new SubsystemComponent(limelight3ASubsystem),
                // Update the light after Limelight has processed the latest camera result.
                new SubsystemComponent(light)
        );

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void onWaitForStart() {
        // Move one route per press, wrapping at either end of the list.
        int index = selectedRoute.ordinal();
        if (gamepad1.dpadRightWasPressed()) {
            selectedRoute = ROUTES[(index + 1) % ROUTES.length];
        } else if (gamepad1.dpadLeftWasPressed()) {
            selectedRoute = ROUTES[(index - 1 + ROUTES.length) % ROUTES.length];
        }

        // Choose alliance independently of the left/right route selection.
        // Use gamepad 1 D-pad up for blue and D-pad down for red.
        // Remember the previous selection so the light changes only when needed.
        Alliance previousAlliance = selectedAlliance;
        if (gamepad1.dpad_up) {
            selectedAlliance = Alliance.BLUE;
        } else if (gamepad1.dpad_down) {
            selectedAlliance = Alliance.RED;
        }

        // Apply the new alliance color directly while waiting for START.
        if (selectedAlliance != previousAlliance) {
            updateAllianceLight();
        }

        // Display both choices so the driver can confirm them before pressing START.
        // START applies this final selection before scheduling autonomous commands.
        telemetry.addData("<== Select route ==>", "D-pad left: previous | right: next");
        telemetry.addData("Selected route", selectedRoute);
        telemetry.addData("Select alliance", "D-pad up: BLUE | down: RED");
        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Apply the driver's final INIT selection before creating autonomous paths.
        applySelectedRoute();
        // Set localization to the selected route's starting coordinates.
        follower.setPose(startPose);

        pickupComplete = false;
        new SequentialGroup(
                new Delay(START_DELAY_SECONDS), // Only the third route has a start delay.
                shootBallsIfPositionIdentified(SHOOTING_RUN_SECONDS),
                // Skip pickup, its completion wait, and the second shot together on the third route.
                selectedRoute == AutoRoute.ROUTE_SHORT
                        ? new InstantCommand(() -> {})
                        : new SequentialGroup(
                                new InstantCommand(() -> schedule(autoRoutinePickBalls())),
                                new WaitUntil(() -> pickupComplete),
                                // Run the second shooting only if the hive is up, limelight can detect the AprilTag.
                                shootBallsIfPositionIdentified(SHOOTING_RUN_SECONDS)
                        ),
                new InstantCommand(() -> schedule(sequential(
                        follow(follower, goToParkAlign()),
                        follow(follower, park())
                )))
        ).schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    private Command autoRoutinePickBalls() {
        return race(
                sequential(
                        instant(intakeSystem::startIntaking).requiring(intakeSystem),
                        follow(follower, goToBalls()),
                        follow(follower, goToBallPick()),
                        follow(follower, returnToStart())
                ),
                waitMs(PICKUP_TIMEOUT_SECONDS * 1000)
        ).then(
                instant(() -> {
                    intakeSystem.stop();
                    pickupComplete = true;
                })
        );
    }

    /**
     * Starts intake, loader, and shooter with 0.5 seconds between each start.
     * Runs all three for runSeconds after the shooter starts, then stops them.
     * Returns a NextFTC command, separate from the Ivy path commands.
     */
    private dev.nextftc.core.commands.Command shootBallsCommand(double runSeconds) {
        if (!Double.isFinite(runSeconds) || runSeconds < 0) {
            throw new IllegalArgumentException("Run duration must be finite and nonnegative");
        }

        return new SequentialGroup(
                shootingSystem.startShooting,
                intakeSystem.startIntake,
                //new Delay(0.5),
                ballLoadingDC.startLoading,
                new Delay(runSeconds),
                new InstantCommand(this::stopSubsystems)
                        .requires(shootingSystem, ballLoadingDC, intakeSystem)
        );
    }

    /**
     * Checks Limelight when this step starts, rather than when the sequence is built.
     * Avoids the constructor bug in NextFTC 1.0.0's IfElseCommand.
     * The shooting command runs inside this step so parking waits for it to finish.
     */
    private dev.nextftc.core.commands.Command shootBallsIfPositionIdentified(double runSeconds) {
        dev.nextftc.core.commands.Command shooting = shootBallsCommand(runSeconds);
        return new dev.nextftc.core.commands.Command() {
            private boolean shouldShoot = false;

            @Override
            public void start() {
                // Decide once on entry; no identified position means skip this shot.
                shouldShoot = limelight3ASubsystem.getIdentifiedShootingPosition() != '0';
                if (shouldShoot) shooting.start();
            }

            @Override
            public void update() {
                // Advance the existing shooting sequence without scheduling it separately.
                if (shouldShoot) shooting.update();
            }

            @Override
            public boolean isDone() {
                // Safe even before start(): no uninitialized selected command is accessed.
                return !shouldShoot || shooting.isDone();
            }

            @Override
            public void stop(boolean interrupted) {
                // Forward completion or cancellation only if shooting actually started.
                if (shouldShoot) shooting.stop(interrupted);
                if (shouldShoot && interrupted) stopSubsystems();
            }
        }.setRequirements(shooting.getRequirements());
    }

    private void stopSubsystems() {
        shootingSystem.stop();
        ballLoadingDC.stop();
        intakeSystem.stop();
    }

    @Override
    public void onStop() {
        if (shootingSystem != null) shootingSystem.stop();
        if (ballLoadingDC != null) ballLoadingDC.stop();
        if (intakeSystem != null) intakeSystem.stop();
    }

    private Path goToBalls() {
        return line(startPose, ballPose).linear(startPose, ballPose);
    }

    private Path goToBallPick() {
        return line(ballPose, ballPickPose).linear(ballPose, ballPickPose);
    }

    private Path returnToStart() {
        return line(ballPickPose, startPose).linear(ballPickPose, startPose);
    }

    private Path goToParkAlign() {
        return line(startPose, parkAlignPose).linear(startPose, parkAlignPose);
    }

    /** Selects the alliance color directly; Light.periodic() applies it to the hardware. */
    private void updateAllianceLight() {
        // Use absolute positions so repeated selections keep the same color.
        if (selectedAlliance == Alliance.BLUE) {
            light.selectColor(Light.SOLID_BLUE_POSITION);
        } else {
            light.selectColor(Light.SOLID_RED_POSITION);
        }
    }

    private Path park(){
        return line(parkAlignPose, parkPose).linear(parkAlignPose, parkPose);
    }
}
