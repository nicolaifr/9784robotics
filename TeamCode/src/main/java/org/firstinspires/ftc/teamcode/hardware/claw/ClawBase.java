package org.firstinspires.ftc.teamcode.hardware.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareBase;

public class ClawBase extends HardwareBase {
    public Servo clawClamp;
    public Servo clawWrist;

    public double currentPos;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        clawClamp = ahwMap.get(Servo.class, "clawClamp");
        clawWrist = ahwMap.get(Servo.class, "clawWrist");
    }

    public void clawClamp(double rightJoystickY, double currentPos, boolean option) {
        if (rightJoystickY >= 0.25 && currentPos < 1 && !option) {
            currentPos += 0.1;
            clawClamp.setPosition(currentPos);
        } else if (rightJoystickY <= -0.25 && currentPos > 0 && !option) {
            currentPos -= 0.1;
            clawClamp.setPosition(currentPos);
        }
    }

    public void clawWrist(double leftJoystickX, double currentPos, boolean option) {
        if (leftJoystickX >= 0.25 && currentPos < 1 && !option) {
            currentPos += 0.1;
            currentPos = this.currentPos;
            clawWrist.setPosition(currentPos);
        } else if (leftJoystickX <= -0.25 && currentPos > 0 && !option){
            currentPos -= 0.1;
            currentPos = this.currentPos;
            clawWrist.setPosition(currentPos);
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
