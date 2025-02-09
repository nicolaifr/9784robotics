package hardware.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import hardware.HardwareBase;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class CameraBase extends HardwareBase {
    public ImageRegion region;


    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

    }
}
