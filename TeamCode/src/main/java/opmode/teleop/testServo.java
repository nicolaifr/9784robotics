package opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServo extends OpMode {

    Servo testServo;
    Servo testServo1;
    Servo wrist;
    Servo clamp;

    public double armPos;
    public double wristPos;

    public int clawState;
    boolean clampClosed;
    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "ArmLeft");
        testServo1 = hardwareMap.get(Servo.class, "ArmRight");
        wrist = hardwareMap.get(Servo.class, "clawClamp");
        clamp = hardwareMap.get(Servo.class, "clawXRotate");

        armPos = 0;
        wristPos = 0;
        clawState = 0;
        clampClosed = false;
    }

    @Override
    public void loop() {
        if (gamepad2.b && armPos != 1) {
            testServo.setPosition(armPos+= 0.001);
            testServo1.setPosition(armPos += 0.001);
        }
        if (gamepad2.a && armPos != 0) {
            testServo.setPosition(armPos -= 0.001);
            testServo1.setPosition(armPos -= 0.001);
        }

        if(gamepad2.dpad_left && wristPos != 1) {
            wrist.setPosition(wristPos += 0.005);
        } else if (gamepad2.dpad_right && wristPos != 0) {
            wrist.setPosition(wristPos -= 0.005);
        }

        switch (clawState) {
            case 0:
                if (gamepad2.right_stick_button) {
                    clawState = 1;
                }
            case 1:
                if (!gamepad2.right_stick_button) {
                    if (!clampClosed) {
                        clamp.setPosition(1);
                    } else {
                        clamp.setPosition(0);
                    }
                    clawState = 0;
                }




        }

    }
}
