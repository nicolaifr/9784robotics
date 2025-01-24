package hardware.slides;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import hardware.HardwareBase;

public class SlidesBase extends HardwareBase {
    public DcMotorEx vertSlides;
    public DcMotorEx horizSlides;
    public DcMotorEx vertSlidesSecond;


    //PID rotate stuff
    private PIDController horizController;
    private PIDController vertController;
    int vertCurrentPos;
    public int horizCurrentPos;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double pH = 0.09, iH = 0, dH = 0.001;
    public static double pV = 0.04, iV = 0, dV = 0.001;
    //feedforward
    public static double Vf = 0.001;
    public static double Hf = 0.2;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = (double) 8192 / 360;

    int vertTarget;
    int horizTarget;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        horizController = new PIDController(pH, iH, dH);
        vertController = new PIDController(pV, iV, dV);

        horizSlides = ahwMap.get(DcMotorEx.class, "horizSlides");
        vertSlides = ahwMap.get(DcMotorEx.class, "vertSlides");
        vertSlidesSecond = ahwMap.get(DcMotorEx.class, "vertSlidesSecond");

        horizSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vertSlidesSecond.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertSlidesSecond.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        vertTarget = 0;
        horizTarget = 0;
    }

    public void verticalSlidesControls(boolean rightTrigger, boolean leftTrigger) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightTrigger) {
            vertSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            vertSlidesSecond.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            vertSlides.setPower(0.5);
            vertSlidesSecond.setPower(0.5);
            vertCurrentPos = vertSlides.getCurrentPosition();
        } else if (leftTrigger) {
            vertSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            vertSlidesSecond.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            vertSlides.setPower(-0.2);
            vertSlidesSecond.setPower(-0.2);
            vertCurrentPos = vertSlides.getCurrentPosition();
        } else {
            setVertTarget(vertCurrentPos);
            vertSlides.setPower(0);
        }
    }

    public void horizSlidesControls(boolean leftT, boolean rightT) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightT) {
            horizTarget = 10;

        } else if (leftT) {
            horizTarget = -200;

            //yes guys we fixed it!
        }
        horizSlides.setTargetPosition(horizTarget);
        horizSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        horizSlides.setPower(1);
    }

    public void PIDF_Vert() {
        vertController.setPID(pV, iV, dV);
        int vertCurrentPos = vertSlides.getCurrentPosition();
        //PID MATH
        double pid = vertController.calculate(vertCurrentPos, vertTarget);
        //feedforward math
        double ff = Math.cos(Math.toRadians(vertTarget / ticks_in_degree)) * Vf;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        vertSlides.setPower(power);
        vertSlidesSecond.setPower(power);
    }

    public void PIDF_H() {
        horizController.setPID(pH, iH, dH);
        int horizCurrentPos = horizSlides.getCurrentPosition();
        //PID MATH
        double pid = horizController.calculate(horizCurrentPos, horizTarget);
        //feedforward math
        double ff = Math.cos(Math.toRadians(horizTarget / ticks_in_degree)) * Hf;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        horizSlides.setPower(power);
    }

    public void setVertTarget(int newTarget) {
        vertTarget = newTarget;
    }

    public void setHorizTarget(int newTarget) {
        horizTarget = newTarget;
    }
}
