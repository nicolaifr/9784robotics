package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class PIDF_Rotate extends OpMode {

    private PIDController controller;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double p = 0.007, i = 0, d = 0.0001;
    //feedforward
    public static double f = 0.15;
    //arm target position
    public static int target = 0;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = ((double) 8192) /360;
    private DcMotorEx armRotate;

    @Override
    public void init() {
        //initialization code when "INIT" is pressed
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armRotate = hardwareMap.get(DcMotorEx.class, "armMotorRotate");
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        controller.setPID(p, i, d);
        int armPos = armRotate.getCurrentPosition();
        //PID MATH
        double pid = controller.calculate(armPos, target);
        //feedforward math
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        //power calculated
        double power = pid + ff;
        //setting motor power after all those calculations
        armRotate.setPower(power);
        //telemetry for tuning
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

    public int getTarget() {
        telemetry.addData("target", target);
        telemetry.update();
        return target;
    }

    public void setTarget(int newTarget){
        target = newTarget;
    }

    public void rotateToPos (int rotateTarget) {
        target = rotateTarget;
        loop();
    }
}
