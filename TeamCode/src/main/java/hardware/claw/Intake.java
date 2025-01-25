package hardware.claw;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import hardware.HardwareBase;

public class Intake extends HardwareBase {
    private PIDController pivotController;
    public static double p = 0.015, i = 0, d = 0.001;
    public static double f = 0.1;
    private final double ticks_in_degree = ((double) 8192) /360;

    int pivotTarget;

    public Servo clawClamp;
    public Servo clawWrist;
    public DcMotor monsterPivot;
    public ColorRangeSensor colorRangeSensor;

    public double clampPos;
    public double wristPos;
    public double pivotPos;
    public int monsterPivotPos;

    public boolean clampOpen;

    int clawState = 0;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

        pivotController = new PIDController(p, i, d);

        clawClamp = ahwMap.servo.get("clamp");
        clawWrist = ahwMap.get(Servo.class, "intakeWrist");
        monsterPivot = ahwMap.get(DcMotor.class, "pivotMotor");
        colorRangeSensor = ahwMap.get(ColorRangeSensor.class, "colorRangeSensor");

        monsterPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        monsterPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clampOpen = false;
        pivotPos = 0;
        wristPos = 0;
        pivotTarget = 0;
    }

    public void clawWrist(boolean b, boolean x) {
        if (b && wristPos != 1) {
            wristPos += 0.03;
            clawWrist.setPosition(wristPos);
        } else if (x && wristPos != 0){
            wristPos -= 0.03;
            clawWrist.setPosition(wristPos);
        }
    }

    public void clawClamp(boolean a) {
//        if (clampOpen) {
//            if (colorRangeSensor.getDistance(DistanceUnit.INCH) < 0.5 && (colorRangeSensor.red() > 200 || colorRangeSensor.blue() > 200)
//            ) {
//                clampOpen = false;
//                closeClaw();
//            }
//        } else {
//            if (a) {
//                openClaw();
//            } else {
//                closeClaw();
//            }
//            clampOpen = !(colorRangeSensor.getDistance(DistanceUnit.INCH) < 0.5 && (colorRangeSensor.red() > 200 || colorRangeSensor.blue() > 200));
//
//        }
        if (a) {
            closeClaw();
            clampOpen = false;
        } else {
            openClaw();
            clampOpen = true;
        }
    }

    public void closeClaw(){
        clawClamp.setPosition(0.3);
        clampOpen = false;
    }
    public void openClaw(){
        clawClamp.setPosition(0.82);
        clampOpen = true;
    }

    public void PIDF_Pivot() {
            pivotController.setPID(p, i, d);
            int currentPos = monsterPivot.getCurrentPosition();
            //PID MATH
            double pid = pivotController.calculate(currentPos, pivotTarget);
            //feedforward math
            double ff = Math.cos(Math.toRadians(pivotTarget/ticks_in_degree)) * f;
            //power calculated
            double power = (pid + ff);
            //setting motor power after all those calculations
            monsterPivot.setPower(Range.clip(power, -0.4, 0.6));
    }

    public void setPivotTarget(int newTarget) {
        pivotTarget = newTarget;
    }

    public void intakeDown(boolean button, boolean button2) {
        if (button && !button2) {
            setPivotTarget(-260);

        } else if (button2 && !button) {
            setPivotTarget(-100);
        }
    }
}
