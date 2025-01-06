package hardware.commands.intothedeep.teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.claw.IntakeClawBase;


public class intakeClawTeleopCommands extends IntakeClawBase {
    public Servo clawClamp;
    public Servo clawWrist;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        clawClamp = ahwMap.get(Servo.class, "clawClamp");
        clawWrist = ahwMap.get(Servo.class, "clawWrist");
    }
}
