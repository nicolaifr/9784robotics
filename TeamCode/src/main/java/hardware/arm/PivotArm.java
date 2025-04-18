package hardware.arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class PivotArm extends HardwareBase {
    public DcMotorEx armRotate;
    public DcMotorEx armExtend;
    public double rotatePos = 0;
    public double extendPos = 0;

//PID rotate stuff
    private PIDController rotateController;
    private PIDController extendController;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double pR = 0.0007, iR = 0, dR = 0.00002;
    //feedforward
    public static double f = 0.01;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = (double) 8192/360;

    double rotateTarget;
    double extendTarget;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

        rotateController = new PIDController(pR, iR, dR);
        extendController = new PIDController(pR, iR, dR);

        rotateController.setPID(pR, iR, dR);
        extendController.setPID(pR, iR, dR);

        armRotate = ahwMap.get(DcMotorEx.class, "armRotate");
        armExtend = ahwMap.get(DcMotorEx.class, "armExtend");

    }

    public void rotateArm(double rightTrigger, double leftTrigger) {
        //code that uses the pidf to do cool sigma stuff
        if (rightTrigger >= 0.9) {
            armRotate.setPower(0.8);
            rotatePos = armRotate.getCurrentPosition();
        } else if (leftTrigger >= 0.9) {
            armRotate.setPower(-0.8);
            rotatePos = armRotate.getCurrentPosition();
        } else {
            setRotateTarget(rotatePos);
            PIDFrotateTo();
        }
    }
    public void extendArm(boolean rightBumper, boolean leftBumper) {
        if (rightBumper) {
            armExtend.setPower(0.8);
            extendPos = armExtend.getCurrentPosition();
        } else if (leftBumper) {
            armExtend.setPower(-0.8);
            extendPos = armExtend.getCurrentPosition();
        } else {
            setExtendTarget(extendPos);
            PIDFextendTo();
        }
    }

    private void setExtendTarget(double extendTarget) {
        this.extendTarget = extendTarget;
    }

    public void PIDFrotateTo() {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        rotateController.setPID(pR, iR, dR);

//        double leftPos = leftEncoderPos();
//        double leftTarget = rotateTarget;
        //PID MATH
//        double leftPID = leftController.calculate(leftPos, leftTarget);
//        //feedforward math
//        double leftFF = Math.cos(Math.toRadians(leftTarget/ticks_in_degree)) * f;
//        //power calculated
//        double leftPower = leftPID + leftFF;
//
        double rightTarget = rotateTarget;
        //PID MATH
        double rightPID = rotateController.calculate(armRotate.getCurrentPosition(), rightTarget);
        //feedforward math
        double rightFF = Math.cos(Math.toRadians(rightTarget/ticks_in_degree)) * f;
        //power calculated
        double rotatePower = rightPID + rightFF;

//        leftPower = Range.clip(leftPower, -1, 1);
        rotatePower = Range.clip(rotatePower, -1, 1);
        //setting motor power after all those calculations
//        armRotateLeft.setPower(rotatePower);
        armRotate.setPower(rotatePower);
//        armRotateRight.setPower(rightPower);

    }

    public void PIDFextendTo() {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        extendController.setPID(pR, iR, dR);

        double extendTarget = this.extendTarget;
        //PID MATH
        double extendPID = extendController.calculate(armExtend.getCurrentPosition(), extendTarget);
        //feedforward math
        double extendFF = Math.cos(Math.toRadians(extendTarget/ticks_in_degree)) * f;
        //power calculated
        double extendPower = extendPID + extendFF;

        extendPower = Range.clip(extendPower, -1, 1);
        //setting motor power after all those calculations
        armExtend.setPower(extendPower);
    }

    public void setRotateTarget(double newTarget) {
        rotateTarget = newTarget;
    }

    public double getRotateTarget() {
        return rotateTarget;
    }

    public void setPID(double p, double i, double d) {
        pR = p;
        iR = i;
        dR = d;
    }
}
