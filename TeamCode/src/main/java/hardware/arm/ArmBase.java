package hardware.arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class ArmBase extends HardwareBase {
    public CRServo armRotateLeft;
    public CRServo armRotateRight;
    public DcMotorEx armRotateLeftEncoder;
    public DcMotorEx armRotateRightEncoder;
    public int rotatePos;
    public int wristPos;

//PID rotate stuff
    private PIDController rotateController;
    private PIDController wristController;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double pR = 0.007, iR = 0, dR = 0.0001;
    public static double pE = 0.008, iE = 0, dE = 0.0001;
    //feedforward
    public static double f = 0.15;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = (double) 8192/360;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        rotateController = new PIDController(pR, iR, dR);
        wristController = new PIDController(pE, iE, dE);

        rotateController.setPID(pR, iR, dR);
        wristController.setPID(pE, iE, dE);

        armRotateLeft = ahwMap.get(CRServo.class, "ArmLeft");
        armRotateRight = ahwMap.get(CRServo.class, "ArmRight");

        armRotateLeftEncoder = ahwMap.get(DcMotorEx.class, "ArmLeftEncoder");
        armRotateRightEncoder = ahwMap.get(DcMotorEx.class, "ArmRightEncoder");
        armRotateLeftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRotateRightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void arm(double rightTrigger, double leftTrigger, boolean rightBumper, boolean leftBumper) {
        //code that uses the pidf to do cool sigma stuff
        if (rightTrigger >= 0.25) {
            armRotateLeft.setPower(rightTrigger);
            armRotateRight.setPower(rightTrigger);
            rotatePos = (armRotateLeftEncoder.getCurrentPosition() + armRotateRightEncoder.getCurrentPosition()) / 2;
        } else if (leftTrigger >= 0.25) {
            armRotateLeft.setPower(leftTrigger);
            armRotateRight.setPower(leftTrigger);
            rotatePos = (armRotateLeftEncoder.getCurrentPosition() + armRotateRightEncoder.getCurrentPosition()) / 2;
        } else if (rightBumper) {
            armRotateLeft.setPower(0.5);
            armRotateRight.setPower(-0.5);
            wristPos = armRotateLeftEncoder.getCurrentPosition() - armRotateRightEncoder.getCurrentPosition();
        } else if (leftBumper) {
            armRotateLeft.setPower(-0.5);
            armRotateRight.setPower(0.5);
            wristPos = armRotateLeftEncoder.getCurrentPosition() - armRotateRightEncoder.getCurrentPosition();
        } else {
//            PIDFrotateTo(rotatePos, wristPos);
        }
    }
    public void PIDFrotateTo(int rotateHoldPos, int wristHoldPos) {

        int armPos = (armRotateLeftEncoder.getCurrentPosition() + armRotateRightEncoder.getCurrentPosition()) / 2;
        //PID MATH
        double rotatePID = rotateController.calculate(armPos, rotateHoldPos);
        //feedforward math
        double rotateFF = Math.cos(Math.toRadians(rotateHoldPos/ticks_in_degree)) * f;
        //power calculated
        double rotatePower = rotatePID + rotateFF;

        int wristPos = armRotateLeftEncoder.getCurrentPosition() - armRotateRightEncoder.getCurrentPosition();
        //PID MATH
        double wristPID = wristController.calculate(wristPos, wristHoldPos);
        //feedforward math
        double wristFF = Math.cos(Math.toRadians(wristHoldPos/ticks_in_degree)) * f;
        //power calculated
        double wristPower = wristPID + wristFF;

        double leftPower = Range.clip(rotatePower + wristPower, -1, 1);
        double rightPower = Range.clip(rotatePower - wristPower, -1, 1);
        //setting motor power after all those calculations
        armRotateLeft.setPower(leftPower);
        armRotateRight.setPower(rightPower);
    }
}
