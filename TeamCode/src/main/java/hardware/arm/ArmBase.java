package hardware.arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class ArmBase extends HardwareBase {
    public DcMotor armRotate;
    public DcMotor armExtend;
    public int rotatePos;
    public int extendPos;
//PID rotate stuff
    private PIDController rotateController;
    private PIDController extendController;
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
        extendController = new PIDController(pE, iE, dE);

        rotateController.setPID(pR, iR, dR);
        extendController.setPID(pE, iE, dE);

        armRotate = ahwMap.get(DcMotor.class, "armMotorRotate");
        armExtend = ahwMap.get(DcMotor.class, "armMotorExtendLeft");

        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armRotateControls(double rightTrigger, double leftTrigger) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightTrigger >= 0.25) {
            armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRotate.setPower(0.8);
            rotatePos = armRotate.getCurrentPosition();
        } else if (leftTrigger >= 0.25) {
            armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRotate.setPower(-0.6);
            rotatePos = armRotate.getCurrentPosition();
        } else {
            PIDFrotateTo(rotatePos);
        }
    }
    public void armExtendControls(boolean leftBumper, boolean rightBumper) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightBumper) {
            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armExtend.setPower(0.8);
            extendPos = armExtend.getCurrentPosition();
        } else if (leftBumper) {
            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armExtend.setPower(-0.8);
            extendPos = armExtend.getCurrentPosition();
        } else {
            PIDFextendTo(extendPos);
        }
    }
    public void PIDFrotateTo(int rotateHoldPos) {

        int armPos = armRotate.getCurrentPosition();
        //PID MATH
        double pid = rotateController.calculate(armPos, rotateHoldPos);
        //feedforward math
        double ff = Math.cos(Math.toRadians(rotateHoldPos/ticks_in_degree)) * f;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        armRotate.setPower(power);
    }

    public void PIDFextendTo(int extendHoldPos) {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        int armPos = armExtend.getCurrentPosition();
        //PID MATH
        double pid = extendController.calculate(armPos, extendHoldPos);
        //feedforward math
        double ff = Math.cos(Math.toRadians(extendHoldPos/ticks_in_degree)) * f;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        armExtend.setPower(power);
    }
}
