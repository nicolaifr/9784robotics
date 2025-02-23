package hardware.arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import hardware.HardwareBase;

public class ArmBase extends HardwareBase {
    public CRServo armRotateLeft;
    public CRServo armRotateRight;
    public AnalogInput armRotateLeftEncoder;
    public AnalogInput armRotateRightEncoder;
    public double rotatePos;
    public double wristPos;

//PID rotate stuff
    private PIDController leftController;
    private PIDController rightController;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double pR = 0.0007, iR = 0, dR = 0.00002;
    //feedforward
    public static double f = 0.01;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = (double) 8192/360;

    double rotateTarget;
    double wristTarget;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

        rightController = new PIDController(pR, iR, dR);
        leftController = new PIDController(pR, iR, dR);

        rightController.setPID(pR, iR, dR);
        leftController.setPID(pR, iR, dR);

        armRotateLeft = ahwMap.get(CRServo.class, "ArmLeft");
        armRotateRight = ahwMap.get(CRServo.class, "ArmRight");

        armRotateLeftEncoder = ahwMap.get(AnalogInput.class, "leftArmEncoder");
        armRotateRightEncoder = ahwMap.get(AnalogInput.class, "rightArmEncoder");

    }

    public double leftEncoderPos() {
        return armRotateLeftEncoder.getVoltage() / 3.3 * 360;
    }

    public double rightEncoderPos() {
        return armRotateRightEncoder.getVoltage() / 3.3 * 360;
    }

    public void arm(double rightTrigger, double leftTrigger, boolean rightBumper, boolean leftBumper) {
        //code that uses the pidf to do cool sigma stuff
        if (rightTrigger >= 0.9) {
            armRotateLeft.setPower(0.8);
            armRotateRight.setPower(-0.8);
            rotatePos = rightEncoderPos();
        } else if (leftTrigger >= 0.9) {
            armRotateLeft.setPower(-0.8);
            armRotateRight.setPower(0.8);
            rotatePos = rightEncoderPos();
        } else {
            setRotateTarget(rotatePos);
            PIDFrotateTo();
        }
    }
    public void PIDFrotateTo() {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        leftController.setPID(pR, iR, dR);
        rightController.setPID(pR, iR, dR);

        double leftPos = leftEncoderPos();
        double leftTarget = rotateTarget;
        //PID MATH
        double leftPID = leftController.calculate(leftPos, leftTarget);
        //feedforward math
        double leftFF = Math.cos(Math.toRadians(leftTarget/ticks_in_degree)) * f;
        //power calculated
        double leftPower = leftPID + leftFF;

        double rightPos = rightEncoderPos();
        double rightTarget = rotateTarget;
        //PID MATH
        double rightPID = rightController.calculate(rightPos, rightTarget);
        //feedforward math
        double rightFF = Math.cos(Math.toRadians(rightTarget/ticks_in_degree)) * f;
        //power calculated
        double rightPower = rightPID + rightFF;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);
        //setting motor power after all those calculations
        armRotateLeft.setPower(leftPower);
        armRotateRight.setPower(leftPower);
//        armRotateRight.setPower(rightPower);

        //telemetry for tuning
        //telemetry for tuning

//        int rotatePos = (leftPos+rightPos)/2;
//        int wristPos = (leftPos-rightPos);
//        telemetry.addData("right pos", rightPos);
//        telemetry.addData("left pos", leftPos);
//        telemetry.addData("right target", rightTarget);
//        telemetry.addData("left target", leftTarget);
//        telemetry.update();
//        telemetry.addData("right power", rightPower);
//        telemetry.addData("left power", leftPower);
    }

    public void setRotateTarget(double newTarget) {
        rotateTarget = newTarget;
    }

    public double getRotateTarget() {
        return rotateTarget;
    }
}
