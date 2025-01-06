package opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServo extends OpMode {

    Servo testServo;
    Servo testServo1;
    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "ArmLeft");
        testServo1 = hardwareMap.get(Servo.class, "ArmRight");
    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            testServo.setPosition(0.5);
            testServo1.setPosition(0.5);
        }
        if (gamepad2.b) {
            testServo.setPosition(1);
            testServo1.setPosition(1);
        }
    }
}
