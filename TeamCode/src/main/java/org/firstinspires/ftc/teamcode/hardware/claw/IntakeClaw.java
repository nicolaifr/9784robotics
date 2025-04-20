package org.firstinspires.ftc.teamcode.hardware.claw;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.HardwareBase;

public class IntakeClaw extends HardwareBase {
    public Servo intake;
    //up down
    public Servo swivel;
    // left right
    public Servo wrist;
    double swivelTarget;
    double wristTarget;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

        swivel = ahwMap.servo.get("swivelServo");
        wrist = ahwMap.servo.get("wristServo");
        intake = ahwMap.get(Servo.class, "intakeServo");
    }

    public void intakeControl(boolean a, boolean b) {
        if (a) {
            intake.setPosition(-1);
        } else if (b) {
            intake.setPosition(1);
        }
    }

    public void swivelControl(boolean dpadUp, boolean dpadDown) {
        if (dpadUp) {
            swivelTarget -= 0.1;
        } else if (dpadDown) {
            swivelTarget += 0.1;
        }
        swivel.setPosition(swivelTarget);
    }

    public void wristControl(boolean dpadLeft, boolean dpadRight) {
        if (dpadLeft) {
            wristTarget -= 0.1;
        } else if (dpadRight) {
            wristTarget += 0.1;
        }
        wrist.setPosition(wristTarget);
    }
}
