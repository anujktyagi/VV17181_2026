package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;


public class Limelight3ASubsystem implements Subsystem {

    private static Limelight3ASubsystem INSTANCE;
    private Telemetry telemetry;

    private Limelight3A vvLimelight;
    LLResult llResult = null;
    // Signed horizontal angle for the same AprilTag shown in telemetry.
    private double tagOffsetDegrees = Double.NaN;
    private static final long MAX_TAG_AGE_MS = 250;
    YawPitchRollAngles orientation = null;
    Pose3D robotPose = null;
    private IMUEx imu = null;
    private double distance = 0.0;
    // scale calculate using mycurvefit.com
    // reference youtube video by Brogan M. Patt (how to measure distance with April tags (limelight 3A)"
    //private final double scale = 29024.85; // Value derived using mycurvefit.com
    private final double scale = 180.51; // Anuj/Team: Reverse engineering scale value derived using scale = knownDistance*SQRT(ta)
    // Use pipeline#8 for Red(TagIDs: 30,31), and #9 for Blue(TagIDs: 32,33)
    private int currentPipeline = 2;
    public char identifiedShootingPosition = '0'; //Zero is default

    private Limelight3ASubsystem(HardwareMap hardwareMap, Telemetry telemetry, IMUEx imu) {
        this.telemetry = telemetry;
        this.imu = imu;
        vvLimelight = hardwareMap.get(Limelight3A.class, "limelight");
        vvLimelight.pipelineSwitch(currentPipeline);
        vvLimelight.start();
    }

    public static Limelight3ASubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry, IMUEx imu) {
        if (INSTANCE == null) {
            INSTANCE = new Limelight3ASubsystem(hardwareMap, telemetry, imu);
        }
        return INSTANCE;
    }

    public static Limelight3ASubsystem getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("You must set up the servo first using telemetry!");
        }
        return INSTANCE;
    }

/*    @Override
    public void initialize() {
        Subsystem.super.initialize();
        vvLimelight.pipelineSwitch(currentPipeline);
        vvLimelight.start();
        telemetry.addData("<===== Limelight Camera Started =====>", "");
    }*/

    @Override
    public void periodic() {
        telemetry.addData("<=== Limelight Camera ===>", " (Pipeline: %d)", currentPipeline);
        orientation = imu.getImu().getRobotYawPitchRollAngles();
        vvLimelight.updateRobotOrientation(orientation.getYaw());
        llResult = vvLimelight.getLatestResult();
        tagOffsetDegrees = Double.NaN; // No angle until this result identifies a tag.
        identifiedShootingPosition = '0'; //Always start with the default position zero

        //Logic help to debug bad/no results
        /*if (llResult == null) {
            telemetry.addData("Limelight Result", "NULL - No data from camera");
        } else if (!llResult.isValid()) {
            telemetry.addData("Limelight Result", "Invalid - No targets detected");
        } else*/
        if(llResult != null && llResult.isValid()) {

            // Logic to get the distance, this is not used right now.
            if (llResult.getTa()>0) {
                // reference youtube video by Brogan M. Pratt (how to measure distance with April tags (limelight 3A)"
                distance = scale/Math.sqrt(llResult.getTa()); //will get values in cm (because scale was calculated with distance in CMs 60,120,180,240)
                distance = distance * 0.393701; //converted to inches
            }
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

            //Identify the very first April Tag and print it.
            if (fiducials!=null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int tagId = fiducial.getFiducialId();  // <-- This is the AprilTag number
                    telemetry.addData("AprilTag ID", tagId);
                    tagOffsetDegrees = fiducial.getTargetXDegrees();
                    identifiedShootingPosition = 'A';
                    break;
                }
            }

            robotPose = llResult.getBotpose_MT2(); // llResult.getBotpose() if need megaTag1 data only. This is configured during the pipeline (MT2 is the yellow bot)

            // Classify this frame into a discrete shooting position based on tx + distance
            //identifiedShootingPosition = classifyShootingPosition(llResult.getTx(), distance);

            String tagAngle = Double.isFinite(tagOffsetDegrees)
                    ? String.format(java.util.Locale.US, "%+.3f°", tagOffsetDegrees)
                    : "No tag";
            telemetry.addData("Limelight", "Tx: %.3f | Ty: %.3f | Ta: %.3f | Tag angle: %s | Yaw: %.3f | Distance: %.3f | Position: %s",
                    llResult.getTx(), llResult.getTy(), llResult.getTa(),
                    tagAngle, robotPose.getOrientation().getYaw(), distance, identifiedShootingPosition);
        }
    }

    /** Returns the tag angle only while the corresponding frame is fresh and on the active pipeline. */
    public double getTagOffsetDegrees() {
        if (llResult == null || llResult.getStaleness() > MAX_TAG_AGE_MS
                || llResult.getPipelineIndex() != currentPipeline) {
            return Double.NaN;
        }
        return tagOffsetDegrees;
    }

    public char getIdentifiedShootingPosition() {
        return identifiedShootingPosition;
    }

    /**
     * Selects either the red-alliance pipeline (8) or the blue-alliance
     * pipeline (9).
     */
    public void switchLinelinePipeline(int newPipeline) {
        currentPipeline = newPipeline;
        vvLimelight.pipelineSwitch(currentPipeline);
        //telemetry.addData("Limelight Pipeline", currentPipeline);
    }

}
