package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.arm.ArmBase;
import org.firstinspires.ftc.teamcode.hardware.claw.Intake;


@TeleOp
@Config
public class ArmTest extends OpMode {
    ArmBase arm;
    Intake intake;
    public static double P;
    public static double I;
    public static double D;

    int clawState;
    @Override
    public void init() {

        arm = new ArmBase();
        intake = new Intake();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        getTelemetry();
        IntakeControls();
        OutTakeControls();
        arm.setPID(P, I, D);
    }
    public void OutTakeControls(){

        arm.arm(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.right_bumper, gamepad2.left_bumper);
    }

    public void IntakeControls() {
        intake.intakeControl(gamepad2.dpad_left, gamepad2.dpad_right);
        intake.swivelControl(gamepad2.dpad_down, gamepad2.dpad_up);
    }

    public void SlidesControls() {
//        slides.verticalSlidesControls(gamepad2.right_stick_button, gamepad2.left_stick_button);
        //slides.PIDF_Vert();
//        slides.PIDF_H();
    }

    public void getTelemetry() {
        telemetry.addData("left position", arm.leftEncoderPos());
        telemetry.addData("armPos", arm.rotatePos);
        telemetry.addData("right position", arm.rightEncoderPos());
        telemetry.update();
    }
}