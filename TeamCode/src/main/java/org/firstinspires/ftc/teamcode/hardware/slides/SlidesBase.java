package org.firstinspires.ftc.teamcode.hardware.slides;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareBase;

public class SlidesBase extends HardwareBase {
    public DcMotor vertSlides;
    public DcMotor horizSlides;


    //PID rotate stuff
    private PIDController horizController;
    private PIDController vertController;
    int vertCurrentPos;
    int horizCurrentPos;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double p = 0, i = 0, d = 0.0001;
    //feedforward
    public static double f = 0;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = (double) 8192/360;

    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        horizController = new PIDController(p, i, d);
        vertController = new PIDController(p, i, d);

        horizSlides = ahwMap.get(DcMotor.class, "horizontalSlides");
        vertSlides = ahwMap.get(DcMotor.class, "verticalSlides");

        horizSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void verticalSlidesControls(double rightTrigger, double leftTrigger) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightTrigger >= 0.25) {
            vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vertSlides.setPower(0.6);
            vertCurrentPos = vertSlides.getCurrentPosition();
        } else if (leftTrigger >= 0.25) {
            vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vertSlides.setPower(-0.6);
            vertCurrentPos = vertSlides.getCurrentPosition();
        } else {
            PIDF_VertTo(vertCurrentPos);
        }
    }
    public void horizSlidesControls(boolean leftBumper, boolean rightBumper) {
        //code that uses the pidf to do cool sigma stuff
        //reversing Y cuz im like pretty sure thats how it is
        if (rightBumper) {
            horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizSlides.setPower(0.75);
            horizCurrentPos = horizSlides.getCurrentPosition();
        } else if (leftBumper) {
            horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizSlides.setPower(-0.75);
            horizCurrentPos = horizSlides.getCurrentPosition();
        } else {
            PIDF_HorizTo(horizCurrentPos);
        }
    }
    public void PIDF_VertTo(int vertTargetPos) {
        vertController.setPID(p, i, d);
        int vertCurrentPos = vertSlides.getCurrentPosition();
        //PID MATH
        double pid = vertController.calculate(vertCurrentPos, vertTargetPos);
        //feedforward math
        double ff = Math.cos(Math.toRadians(vertTargetPos/ticks_in_degree)) * f;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        vertSlides.setPower(power);
    }

    public void PIDF_HorizTo(int horizTargetPos) {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        horizController.setPID(p, i, d);
        int horizCurrentPos = horizSlides.getCurrentPosition();
        //PID MATH
        double pid = horizController.calculate(horizCurrentPos, horizTargetPos);
        //feedforward math
        double ff = Math.cos(Math.toRadians(horizTargetPos/ticks_in_degree)) * f;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        horizSlides.setPower(power);
    }
}
