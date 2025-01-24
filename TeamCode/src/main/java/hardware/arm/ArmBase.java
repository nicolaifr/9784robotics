package hardware.arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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
    public DcMotorEx armRotateLeftEncoder;
    public DcMotorEx armRotateRightEncoder;
    public int rotatePos;
    public int wristPos;

//PID rotate stuff
    private PIDController leftController;
    private PIDController rightController;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double pR = 0.0004, iR = 0, dR = 0.00002;
    //feedforward
    public static double f = -0.05;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = (double) 8192/360;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

        rightController = new PIDController(pR, iR, dR);
        leftController = new PIDController(pR, iR, dR);

        rightController.setPID(pR, iR, dR);
        leftController.setPID(pR, iR, dR);

        armRotateLeft = ahwMap.get(CRServo.class, "ArmLeft");
        armRotateRight = ahwMap.get(CRServo.class, "ArmRight");

        armRotateLeftEncoder = ahwMap.get(DcMotorEx.class, "leftFrontWheel");
        armRotateRightEncoder = ahwMap.get(DcMotorEx.class, "rightFrontWheel");

        armRotateLeftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRotateRightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRotateLeftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armRotateRightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void arm(double rightTrigger, double leftTrigger, boolean rightBumper, boolean leftBumper) {
        //code that uses the pidf to do cool sigma stuff
        if (rightTrigger >= 0.9) {
            armRotateLeft.setPower(0.2);
            armRotateRight.setPower(0.2);
            rotatePos = (-armRotateLeftEncoder.getCurrentPosition() - armRotateRightEncoder.getCurrentPosition()) / 2;
        } else if (leftTrigger >= 0.9) {
            armRotateLeft.setPower(-0.2);
            armRotateRight.setPower(-0.2);
            rotatePos = (-armRotateLeftEncoder.getCurrentPosition() - armRotateRightEncoder.getCurrentPosition()) / 2;
        } else if (rightBumper) {
            armRotateLeft.setPower(0.1);
            armRotateRight.setPower(-0.1);
            wristPos = -armRotateLeftEncoder.getCurrentPosition() + armRotateRightEncoder.getCurrentPosition();
        } else if (leftBumper) {
            armRotateLeft.setPower(-0.1);
            armRotateRight.setPower(0.1);
            wristPos = -armRotateLeftEncoder.getCurrentPosition() + armRotateRightEncoder.getCurrentPosition();
        } else {
            PIDFrotateTo(rotatePos, wristPos);
        }
    }
    public void PIDFrotateTo(int rotateTarget, int wristTarget) {
        leftController.setPID(pR, iR, dR);
        rightController.setPID(pR, iR, dR);

        int leftPos = -armRotateLeftEncoder.getCurrentPosition();
        double leftTarget = rotateTarget - (double) wristTarget / 2;
        //PID MATH
        double leftPID = leftController.calculate(leftPos, leftTarget);
        //feedforward math
        double leftFF = Math.cos(Math.toRadians(leftTarget/ticks_in_degree)) * f;
        //power calculated
        double leftPower = Range.clip(leftPID + leftFF, -1, 1);

        int rightPos = -armRotateRightEncoder.getCurrentPosition();
        double rightTarget = rotateTarget + (double) wristTarget / 2;
        //PID MATH
        double rightPID = rightController.calculate(rightPos, rightTarget);
        //feedforward math
        double rightFF = Math.cos(Math.toRadians(rightTarget/ticks_in_degree)) * f;
        //power calculated
        double rightPower = Range.clip(rightPID + rightFF, -1, 1);

        //setting motor power after all those calculations
        armRotateLeft.setPower(leftPower);
        armRotateRight.setPower(rightPower);
        //telemetry for tuning

//        int rotatePos = (leftPos+rightPos)/2;
//        int wristPos = (leftPos-rightPos);
        telemetry.addData("right pos", rightPos);
        telemetry.addData("left pos", leftPos);
        telemetry.addData("right target", rightTarget);
        telemetry.addData("left target", leftTarget);
//        telemetry.addData("right power", rightPower);
//        telemetry.addData("left power", leftPower);
    }
}
