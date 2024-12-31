package hardware.commands.intothedeep.auto;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.slides.SlidesBase;

public class slidesAutoCommands extends SlidesBase {
    public DcMotorEx horiz;
    public DcMotorEx vertical;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        horiz = ahwMap.get(DcMotorEx.class, "horizontalSlides");
        vertical = ahwMap.get(DcMotorEx.class, "verticalSlides");
    }
}
