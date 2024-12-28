package org.firstinspires.ftc.teamcode.hardware.commands.intothedeep.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.claw.ClawBase;

public class clawAutoCommands extends ClawBase {
    public Servo clawClamp;
    public Servo clawWrist;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        clawClamp = ahwMap.get(Servo.class, "clawClamp");
        clawWrist = ahwMap.get(Servo.class, "clawWrist");
    }

}
