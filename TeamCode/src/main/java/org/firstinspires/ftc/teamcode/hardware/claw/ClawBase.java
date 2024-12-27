package org.firstinspires.ftc.teamcode.hardware.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareBase;

public class ClawBase extends HardwareBase {
    public Servo clawClamp;
    public Servo clawWrist;

    public double clampPos;
    public double wristPos;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        clawClamp = ahwMap.get(Servo.class, "clawClamp");
        clawWrist = ahwMap.get(Servo.class, "clawWrist");
    }

    public void clawClamp(double rightJoystickY, boolean option) {
        if (rightJoystickY >= 0.25 && clampPos < 1 && !option) {
            clampPos += 0.1;
            clawClamp.setPosition(clampPos);
        } else if (rightJoystickY <= -0.25 && clampPos > 0 && !option) {
            clampPos -= 0.1;
            clawClamp.setPosition(clampPos);
        }
    }

    public void clawWrist(double leftJoystickX, boolean option) {
        if (leftJoystickX >= 0.25 && wristPos < 1 && !option) {
            wristPos += 0.1;
            clawWrist.setPosition(wristPos);
        } else if (leftJoystickX <= -0.25 && wristPos > 0 && !option){
            wristPos -= 0.1;
            clawWrist.setPosition(wristPos);
        }
    }

    public void closeClaw(){
        clawClamp.setPosition(1);
    }
    public void openClaw(){
        clawClamp.setPosition(0);
    }
    public void setClampPos(double clampPos) {
        clawClamp.setPosition(clampPos);
    }
    public void setWristPos(double wristPos){
        clawWrist.setPosition(wristPos);
    }
}
