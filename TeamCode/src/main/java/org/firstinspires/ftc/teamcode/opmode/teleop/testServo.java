package opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class testServo extends OpMode {
    private CRServo testServo1;

    private CRServo testServo2;
    public static double servoPos = 0.0;

    @Override
    public void init() {
        testServo1 = hardwareMap.get(CRServo.class, "ArmLeft");
        testServo2 = hardwareMap.get(CRServo.class, "ArmRight");

    }

    @Override
    public void loop() {
//        if (gamepad2.a) {
//            testServo1.setPosition(0.0);
//            testServo2.setPosition(0.0);
//
////            testServo1.setPosition(0.7);
////            testServo2.setPosition(0.7);
//        }
//        if (gamepad2.b) {
//            testServo1.setPosition(0.3);
//            testServo2.setPosition(0.3);
//        }


        testServo1.setPower(servoPos);
        testServo2.setPower(servoPos);

    }
}
