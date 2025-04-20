package org.firstinspires.ftc.teamcode.hardware.armclaw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareBase;

public class ClawCommands extends HardwareBase {
    public Servo clawClamp;
    public Servo clawWrist;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        clawClamp = ahwMap.get(Servo.class, "clawClamp");
        clawWrist = ahwMap.get(Servo.class, "clawWrist");
    }

    public void closeClaw(){
        clawClamp.setPosition(0);
    }
    public void openClaw() {
        clawClamp.setPosition(1);
    }
}
