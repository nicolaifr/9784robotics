package hardware.claw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class IntakeClawBase extends HardwareBase {
    public Servo clawClamp;
    public Servo clawWrist;
    public DcMotor monsterPivot;
    public Servo miniPivot;

    public double clampPos;
    public double wristPos;

    public boolean clampOpen;

    int clawState = 0;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        clawClamp = ahwMap.get(Servo.class, "clawClamp");
        clawWrist = ahwMap.get(Servo.class, "clawWrist");
        clampOpen = false;
    }

    public void clawWrist(double rightJoystickX) {
        if (rightJoystickX >= 0.25 && wristPos < 1) {
            wristPos += 0.05;
            clawWrist.setPosition(wristPos);
        } else if (rightJoystickX <= -0.25 && wristPos > 0){
            wristPos -= 0.05;
            clawWrist.setPosition(wristPos);
        }
    }

    public void clawClamp(boolean rightStickButton) {
        switch (clawState) {
            case 0: // waits for gamepad2.a to be pressed
                if (rightStickButton) {
                    clawState = 1;
                }
                break;
            case 1: // opens/closes claw when gamepad2.a is released
                if (!rightStickButton) {
                    if (clampOpen) {
                        closeClaw();
                    } else {
                        openClaw();
                    }
                    clawState = 0;
                }
        }
    }

    public void closeClaw(){
        clawClamp.setPosition(1);
        clampOpen = false;
    }
    public void openClaw(){
        clawClamp.setPosition(0.4);
        clampOpen = true;
    }
    public void setClampPos(double clampPos) {
        clawClamp.setPosition(clampPos);
    }
    public void setWristPos(double wristPos){
        clawWrist.setPosition(wristPos);
    }
}
