package hardware.claw;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.HardwareBase;

public class Intake extends HardwareBase {
    public static double p = 0.01, i = 0, d = 0.000;
    public static double f = 0.1;
    private final double ticks_in_degree = ((double) 8192) /360;
    int swivelTarget;
    public Servo leftSwivel;
    public Servo rightSwivel;
    public DcMotorEx intake;
    public ColorRangeSensor colorRangeSensor;
    public double swivelPos;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

        leftSwivel = ahwMap.servo.get("swivelLeft");
        rightSwivel = ahwMap.servo.get("swivelRight");
        intake = ahwMap.get(DcMotorEx.class, "intakeMotor");
        colorRangeSensor = ahwMap.get(ColorRangeSensor.class, "colorRangeSensor");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        swivelPos = 0;
        swivelTarget = 0;
    }

    public void intakeControl(boolean a, boolean b) {
        if (a) {
            intakeSpec();
        } else if (b) {
            outakeSpec();
        } else {
            keepSpec();
        }
    }
    public void intakeSpec(){
        intake.setPower(0.8);
    }
    public void outakeSpec(){
        intake.setPower(-0.8);
    }

    public void keepSpec() {
        intake.setPower(0);
    }

    public void setSwivelTarget(double newTarget) {
        rightSwivel.setPosition(-newTarget);
        leftSwivel.setPosition(newTarget);
    }

    public void swivelControl(boolean button, boolean button2) {
        if (button && !button2) {
            setSwivelTarget(0.5);

        } else if (button2 && !button) {
            setSwivelTarget(-1);
        }
    }
}
