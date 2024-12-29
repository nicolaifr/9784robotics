package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class telemetryEncoders extends OpMode {
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBackWheel");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackWheel");
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontWheel");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontWheel");

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("leftBack pos", leftBack.getCurrentPosition());
        telemetry.addData("rightBack pos", rightBack.getCurrentPosition());
        telemetry.addData("leftFront pos", leftFront.getCurrentPosition());
        telemetry.addData("rightFront pos", rightFront.getCurrentPosition());
        telemetry.update();
    }
}
