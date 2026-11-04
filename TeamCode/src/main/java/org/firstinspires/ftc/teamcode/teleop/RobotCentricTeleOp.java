/* This is updated code as of 10/19/2025 */
package org.firstinspires.ftc.teamcode.teleop;

// These are names we made for each wheel motor in another file (ChassisConstants)
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_REAR_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// These are the “subsystems” — little robot parts that do special jobs
import org.firstinspires.ftc.teamcode.subsystems.BallLoadingDC;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Limelight3ASubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

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

    // These are the robot's different systems (we’ll use them later)
    private ShootingSystem shootingSystem;
    //private ShootingDirectionServo shootingDirectionServo;
    private BallLoadingDC ballLoadingDC;
    private Intake intakeSystem;
    private Limelight3ASubsystem limelight3ASubsystem;
    private Light light;
    protected final IMUEx imu = new IMUEx("imu", Direction.DOWN, Direction.FORWARD).zeroed();

    // This is the "constructor" — runs once when the program starts loading
    public RobotCentricTeleOp() {

        // We connect our subsystems to the robot and give them access to telemetry (data shown on driver station)
        shootingSystem = ShootingSystem.getInstance(telemetry);
        intakeSystem = Intake.getInstance(telemetry);
        //shootingDirectionServo = ShootingDirectionServo.getInstance(telemetry);
        ballLoadingDC = BallLoadingDC.getInstance(telemetry);
        light = Light.getInstance(telemetry);
    }

    @Override
    public void onInit() {
        super.onInit();
    
        limelight3ASubsystem = Limelight3ASubsystem.getInstance(this.hardwareMap, telemetry, imu);

        // Here we "add" all these subsystems so NextFTC can manage and update them automatically
        addComponents(
                new SubsystemComponent(limelight3ASubsystem),
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(intakeSystem),
                new SubsystemComponent(ballLoadingDC),
                new SubsystemComponent(light),
                BulkReadComponent.INSTANCE,   // reads all sensors at once for faster updates
                BindingsComponent.INSTANCE    // helps connect buttons on the gamepads to commands
        );

    }

    // These are our 4 drive motors (the robot’s wheels)
    // The MotorEx class is like a smarter motor object from the NextFTC library
    private final MotorEx frontLeftMotor = new MotorEx(LEFT_FRONT_MOTOR_NAME);
    private final MotorEx frontRightMotor = new MotorEx(RIGHT_FRONT_MOTOR_NAME).reversed(); // reversed so both sides move forward correctly
    private final MotorEx backLeftMotor = new MotorEx(LEFT_REAR_MOTOR_NAME);
    private final MotorEx backRightMotor = new MotorEx(RIGHT_REAR_MOTOR_NAME).reversed();

    // This method runs when the driver presses the START button on the Driver Station
    @Override
    public void onStartButtonPressed() {

        // This tells the robot how to drive using mecanum wheels (which move in all directions)
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().rightStickY(),  // forward/backward
                Gamepads.gamepad1().rightStickX(),  // strafe left/right
                Gamepads.gamepad1().leftStickX()  // turn left/right
        );

        // "Schedule" means start running that drive command
        driverControlled.schedule();

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
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(
                new SequentialGroup(
                        light.setPattern(0.05),
                        new InstantCommand(() -> {
                            limelight3ASubsystem.switchLinelinePipeline(9);
                        }).requires(limelight3ASubsystem)
                )
        );

        // D-Pad Right → stop the ball loading servo
        //Gamepads.gamepad2().dpadDown().whenBecomesTrue(light.setPattern(Light.SOLID_RED_POSITION));
        Gamepads.gamepad2().dpadDown().whenBecomesTrue(
                new SequentialGroup(
                        light.setPattern(-0.05),
                        new InstantCommand(() -> {
                            limelight3ASubsystem.switchLinelinePipeline(8);
                        }).requires(limelight3ASubsystem)
                )
        );
    }

}
