package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "ALLINONNIGGADOLPH";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(2.5, -3.5, Math.PI);
        OTOSConstants.linearScalar = 1.18;
        OTOSConstants.angularScalar = 0.98;

    }
}




