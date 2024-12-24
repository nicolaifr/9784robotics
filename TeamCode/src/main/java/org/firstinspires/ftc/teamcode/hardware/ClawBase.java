package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public void clawClamp(double rightJoystickY, double currentPos = 0) {
        if (rightJoystickY >= 0.25 && currentPos < 1) {
            currentPos += 0.1;
            clawClamp.setPosition(currentPos);
        } else if (rightJoystickY <= -0.25 && currentPos > 0) {
            currentPos -= 0.1;
            clawClamp.setPosition(currentPos);
        }
    }

    public void clawWrist(double leftJoystickX, double currentPos) {
        if (leftJoystickX >= 0.25 && currentPos < 1) {
            currentPos += 0.1;
            currentPos = this.currentPos;
            clawWrist.setPosition(currentPos);
        } else if (leftJoystickX <= -0.25 && currentPos > 0){
            currentPos -= 0.1;
            currentPos = this.currentPos;
            clawWrist.setPosition(currentPos);
        }
    }
}
