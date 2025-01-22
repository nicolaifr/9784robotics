package hardware.claw;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class OutTake extends HardwareBase {

    Servo testServo;
    Servo testServo1;
    Servo wrist;
    Servo clamp;

    public double armPos;
    public double wristPos;

    public int clawState;
    boolean clampClosed;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        wrist = ahwMap.get(Servo.class, "clawClamp");
        clamp = ahwMap.get(Servo.class, "clawXRotate");

        armPos = 0;
        wristPos = 0;
        clawState = 0;
        clampClosed = false;
    }
    public void clamp(boolean rb, boolean lb){
        if (rb) {
            clamp.setPosition(0.9);
        } else if (lb) {
            clamp.setPosition(0.1);
        }
    }
    public void arm(boolean a, boolean b) {
        if (b) {
            testServo.setPosition(armPos+= 0.003);
            testServo1.setPosition(armPos += 0.003);
        }
        if (a) {
            testServo.setPosition(armPos -= 0.003);
            testServo1.setPosition(armPos -= 0.003);
        }
    }

    public void wrist(double rt) {
        if(rt >= 0.1 && wristPos <= 1) {
            wristPos += 0.003;
            wrist.setPosition(wristPos);
        } else if (rt <= -0.1 && wristPos >= 0) {
            wristPos -= 0.003;
            wrist.setPosition(wristPos);
        }
    }
}
