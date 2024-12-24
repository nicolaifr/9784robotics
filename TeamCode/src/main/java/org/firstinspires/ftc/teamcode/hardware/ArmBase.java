package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmBase extends HardwareBase {
    public DcMotor armRotate;
    public DcMotor armExtend;


//PID rotate stuff
    private PIDController rotateController;
    private PIDController extendController;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double p = 0.007, extendP = 0.008, i = 0, d = 0.0001;
    //feedforward
    public static double f = 0.15;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = (double) 8192 /360;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        rotateController = new PIDController(p, i, d);
        extendController = new PIDController(extendP, i, d);

        armRotate = ahwMap.get(DcMotor.class, "armMotorRotate");
        armExtend = ahwMap.get(DcMotor.class, "armMotorExtendLeft");

        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armRotateControls(double rightTrigger, double leftTrigger, int rotatePos) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightTrigger >= 0.25) {
            armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRotate.setPower(0.6);
        } else if (leftTrigger >= 0.25) {
            armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armRotate.setPower(-0.6);
        } else {
            PIDFrotateTo(rotatePos);
        }
    }
    public void armExtendControls(boolean leftBumper, boolean rightBumper, int extendPos) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightBumper) {
            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armExtend.setPower(0.75);
        } else if (leftBumper) {
            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armExtend.setPower(-0.75);
        } else {
            PIDFextendTo(extendPos);
        }
    }
    public void PIDFrotateTo(int rotateHoldPos) {
        rotateController.setPID(p, i, d);
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
        extendController.setPID(extendP, i, d);
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
