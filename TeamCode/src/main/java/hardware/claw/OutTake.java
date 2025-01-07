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
        testServo = ahwMap.get(Servo.class, "ArmLeft");
        testServo1 = ahwMap.get(Servo.class, "ArmRight");
        wrist = ahwMap.get(Servo.class, "clawClamp");
        clamp = ahwMap.get(Servo.class, "clawXRotate");

        armPos = 0;
        wristPos = 0;
        clawState = 0;
        clampClosed = false;
    }
    public void clamp(boolean rb, boolean lb){
        if(rb) {
            clamp.setPosition(1);
        } else if (lb) {
            clamp.setPosition(0);
        }
    }
    public void arm(boolean a, boolean b) {
        if (b && armPos != 1) {
            testServo.setPosition(armPos+= 0.001);
            testServo1.setPosition(armPos += 0.001);
        }
        if (a && armPos != 0) {
            testServo.setPosition(armPos -= 0.001);
            testServo1.setPosition(armPos -= 0.001);
        }
    }

    public void wrist(double rt, double lt) {
        if(rt >= 0.5 && wristPos != 1) {
            wrist.setPosition(wristPos += 0.003);
        } else if (lt >= 0.5 && wristPos != 0) {
            wrist.setPosition(wristPos -= 0.003);
        }
    }
}
