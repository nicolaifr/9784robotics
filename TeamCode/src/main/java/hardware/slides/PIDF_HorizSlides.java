package hardware.slides;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDF_HorizSlides extends OpMode {

    private PIDController controller;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double p = 0.07, i = 0, d = 0.001;
    //feedforward
    public static double f = 0.3;
    //arm target position
    public static int target = 0;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = 8192/360;
    private DcMotorEx horizSlides;

    @Override
    public void init() {
        //initialization code when "INIT" is pressed
        controller = new PIDController(p, i, d);

        horizSlides = hardwareMap.get(DcMotorEx.class, "horizSlides");
        horizSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        controller.setPID(p, i, d);
        int slidePos = horizSlides.getCurrentPosition();
        //PID MATH
        double pid = controller.calculate(slidePos, target);
        //feedforward math
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        horizSlides.setPower(power);
        //telemetry for tuning
        getTelemetry();
    }

    public int getTarget() {
        return target;
    }

    public void setTarget(int newTarget){
        target = newTarget;
    }

    public void getTelemetry(){
        telemetry.addData("location", horizSlides.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("poewr", horizSlides.getPower());
        telemetry.update();

    }
}
