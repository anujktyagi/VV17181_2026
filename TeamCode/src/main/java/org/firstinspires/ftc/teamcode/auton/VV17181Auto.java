package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
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
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class VV17181Auto extends NextFTCOpMode {

    // Names for autonomous routes from the straight and side starting positions.
    private enum AutoRoute {
        ROUTE_STRAIGHT,
        ROUTE_SIDE
    }

    // Default to the straight route until the driver selects another during INIT.
    private AutoRoute selectedRoute = AutoRoute.ROUTE_STRAIGHT;

    // Alliance choice is selected during INIT; it does not mirror route coordinates yet.
    private enum Alliance {
        BLUE,
        RED
    }

    // Default to blue unless the driver selects red before pressing START.
    private Alliance selectedAlliance = Alliance.BLUE;

    private static double SHOOTING_RUN_SECONDS = 8.0;

    private Follower follower;
    private boolean pickupComplete;
    private ShootingSystem shootingSystem;
    private BallLoadingDC ballLoadingDC;
    private Intake intakeSystem;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Active poses are assigned from the selected route before they are used.
    private Pose startPose;
    private Pose ballPose;
    private Pose ballPickPose;
    private Pose parkAlignPose;
    private Pose parkPose;

    /** Assigns inches and degrees relative to the selected route's starting position. */
    private void applySelectedRoute() {
        // Separate cases allow each route's poses to be tuned independently.
        switch (selectedRoute) {
            case ROUTE_STRAIGHT:
                // Straight route: coordinates relative to its starting position.
                startPose = poseFactory.of(0, 0, 0);
                ballPose = poseFactory.of(-4, -32, 0);
                ballPickPose = poseFactory.of(0, -32, 0);
                parkAlignPose = poseFactory.of(-6, -24, 45);
                parkPose = poseFactory.of(-67, -40, 90);
                break;
            case ROUTE_SIDE:
                // Side route: intentionally identical for now; adjust these values later.
                startPose = poseFactory.of(0, 0, 0);
                ballPose = poseFactory.of(2, -16, 27);
                ballPickPose = poseFactory.of(4, -14, 27);
                parkAlignPose = poseFactory.of(-14, -10, 60);
                parkPose = poseFactory.of(-54, -50, 112);
                SHOOTING_RUN_SECONDS = 10.0;
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

        addComponents(
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(ballLoadingDC),
                new SubsystemComponent(intakeSystem)
        );

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void onWaitForStart() {
        // Select a route during INIT without starting any robot movement.
        if (gamepad1.dpad_left) {
            selectedRoute = AutoRoute.ROUTE_SIDE;
        } else if (gamepad1.dpad_right) {
            selectedRoute = AutoRoute.ROUTE_STRAIGHT;
        }

        // Choose alliance independently of the left/right route selection.
        // Use gamepad 1 D-pad up for blue and D-pad down for red.
        if (gamepad1.dpad_up) {
            selectedAlliance = Alliance.BLUE;
        } else if (gamepad1.dpad_down) {
            selectedAlliance = Alliance.RED;
        }

        // Display both choices so the driver can confirm them before pressing START.
        // START applies this final selection before scheduling autonomous commands.
        telemetry.addData("<== Select route ==>", "D-pad left: SIDE | right: STRAIGHT");
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
                shootBallsCommand(SHOOTING_RUN_SECONDS),
                new InstantCommand(() -> schedule(autoRoutinePickBalls())),
                new WaitUntil(() -> pickupComplete),
                shootBallsCommand(SHOOTING_RUN_SECONDS),
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
        return sequential(
                instant(intakeSystem::startIntaking).requiring(intakeSystem),
                follow(follower, goToBalls()),
                follow(follower, goToBallPick()),
                follow(follower, returnToStart()),
                instant(intakeSystem::stop).requiring(intakeSystem),
                instant(() -> pickupComplete = true)
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

    private Path park(){
        return line(parkAlignPose, parkPose).linear(parkAlignPose, parkPose);
    }
}
