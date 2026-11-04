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

    private static final double SHOOTING_RUN_SECONDS = 7.0;

    private Follower follower;
    private boolean pickupComplete;
    private ShootingSystem shootingSystem;
    private BallLoadingDC ballLoadingDC;
    private Intake intakeSystem;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose ballPose = poseFactory.of(18, -6, 0);
    private final Pose ballPickPose = poseFactory.of(24, -6, 0);
    private final Pose parkAlignPose = poseFactory.of(24, 0, 45);
    private final Pose parkPose = poseFactory.of(-42, -6, 90);


    @Override
    public void onInit() {
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
    public void onStartButtonPressed() {
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
                intakeSystem.startIntake,
                //new Delay(4.0),
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
