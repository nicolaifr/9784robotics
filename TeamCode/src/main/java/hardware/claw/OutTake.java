package hardware.claw;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class OutTake extends HardwareBase {

    Servo testServo;
    Servo testServo1;
    public Servo wrist;
    public Servo clamp;

    public double armPos;
    public double wristPos;

    public int clawState;
    boolean clampClosed;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        wrist = ahwMap.get(Servo.class, "clawClamp");
        clamp = ahwMap.get(Servo.class, "clawRotate");

        armPos = 0;
        wristPos = 0;
        clawState = 0;
        clampClosed = false;
    }
    public void clamp(boolean rb, boolean lb){
        if (rb) {
            clamp.setPosition(1);
        } else if (lb) {
            clamp.setPosition(0);
        }
    }

    public void wrist(boolean leftButton, boolean rightButton) {
        if(rightButton && wristPos <= 1) {
            wristPos += 0.03;
            wrist.setPosition(wristPos);
        } else if (leftButton && wristPos >= -1) {
            wristPos -= 0.03;
            wrist.setPosition(wristPos);
        }
//        wrist.setPosition(1);
    }

    public void closeClamp() {
        clamp.setPosition(1);
    }
    public void openClamp() {
        clamp.setPosition(0);
    }
}
