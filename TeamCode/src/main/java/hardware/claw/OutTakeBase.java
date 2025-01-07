package hardware.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class OutTakeBase extends HardwareBase {
    //all persepctives taken like ur giving diddly backshots to the robot (from behind)
    public Servo ArmLeft;
    public Servo ArmRight;
    public Servo clawClamp;
    public Servo clawYRotate;
    public Servo clawXRotate;
    public boolean clampOpen;
    public int clawState;
    public double wristPos;
    public double armPos;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        clawClamp = ahwMap.get(Servo.class, "clawXRotate");
        ArmLeft = ahwMap.get(Servo.class, "ArmLeft");
        ArmRight = ahwMap.get(Servo.class, "ArmRight");
        clawXRotate = ahwMap.get(Servo.class, "clawClamp");


        clampOpen = false;
        clawState = 0;
        armPos = ArmLeft.getPosition();
        wristPos = clawXRotate.getPosition();
    }

    public void closeClaw(){
        clawClamp.setPosition(0.4);
        clampOpen = false;
    }
    public void openClaw(){
        clawClamp.setPosition(1);
        clampOpen = true;
    }
    public void clawClamp(boolean leftStickButton) {
        switch (clawState) {
            case 0: // waits for gamepad2.a to be pressed
                if (leftStickButton) {
                    clawState = 1;
                }
                break;
            case 1: // opens/closes claw when gamepad2.a is released
                if (!leftStickButton) {
                    if (clampOpen) {
                        closeClaw();
                    } else {
                        openClaw();
                    }
                    clawState = 0;
                }
        }
    }

    public void diffyArm(double leftStickY) {
        if (leftStickY <= -0.25 && armPos < 1) {
            armPos += 0.05;
            ArmRight.setPosition(armPos);
            ArmLeft.setPosition(armPos);
        } else if (leftStickY >= 0.25 && armPos > 0){
            armPos -= 0.05;
            ArmRight.setPosition(armPos);
            ArmLeft.setPosition(armPos);
        }
    }
    public void clawWrist(double leftStickX) {
        if (leftStickX <= -0.25 && wristPos < 1) {
            wristPos += 0.05;
            ArmRight.setPosition(wristPos);
            ArmLeft.setPosition(wristPos);
        } else if (leftStickX >= 0.25 && wristPos > 0){
            wristPos -= 0.05;
            ArmRight.setPosition(wristPos);
            ArmLeft.setPosition(wristPos);
        }
    }

}
