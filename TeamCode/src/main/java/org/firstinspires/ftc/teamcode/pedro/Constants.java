package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLM");
        c.frontRightName.set("FRM");
        c.backLeftName.set("RLM");
        c.backRightName.set("RRM");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        //Anuj added this manually. Also recommended on pedropathig.com, mecanum tuning process.
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.1333300372746986);
        c.yPodOffset.set(3.73031045508197);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.8930008279389948);
                Controller secondaryTranslationalForward = Controller.proportional(0.32993994450187397);
                Controller primaryTranslationalLateral = Controller.proportional(0.4838852723528427);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17878267848176924);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.02729944991006572));
                c.brake.set(Controller.proportionalFeedforward(0.02320453242355586));

                c.headingFeedback.set(Controller.proportional(4.384479838516467));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04055025631630997, 0.009680103706936737));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06913893221971718, 0.028760526333317073));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0027596139879325367, 0.004779479198267454));

                c.maxAchievableForwardVelocity.set(37.018523275310805);
                c.maxAchievableStrafeVelocity.set(27.771094076412606);
                c.naturalForwardDeceleration.set(28.3129160755326);
                c.naturalStrafeDeceleration.set(44.03709460503653);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight( foresightConfig)
        );
    }
}