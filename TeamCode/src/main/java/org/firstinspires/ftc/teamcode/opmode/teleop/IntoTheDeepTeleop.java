package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.arm.ArmBase;
import org.firstinspires.ftc.teamcode.hardware.claw.Intake;
import org.firstinspires.ftc.teamcode.hardware.claw.PIDF_Pivot;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveTrainBase;
import org.firstinspires.ftc.teamcode.hardware.claw.OutTake;
import org.firstinspires.ftc.teamcode.hardware.slides.SlidesBase;
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;


@TeleOp
public class IntoTheDeepTeleop extends OpMode {
    DriveTrainBase drive;
    OutTake outtake;
    SlidesBase slides;
    Intake intake;
    ArmBase arm;


    int clawState;
    @Override
    public void init() {

        outtake = new OutTake();
        intake = new Intake();
        slides = new SlidesBase();
        arm = new ArmBase();
        drive = new DriveTrainBase();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        outtake.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        slides.init(hardwareMap, telemetry);
        drive.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        getTelemetry();
        OutTakeControls();
        IntakeControls();
        SlidesControls();
    }
    public void OutTakeControls(){
        outtake.clamp(gamepad2.x, gamepad2.y);
//        outtake.wrist(gamepad2.b, gamepad2.a);
//        arm.arm(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.right_bumper, gamepad2.left_bumper);
    }

    public void IntakeControls() {
        intake.intakeControl(gamepad2.dpad_left, gamepad2.dpad_right);
        intake.swivelControl(gamepad2.dpad_down, gamepad2.dpad_up);
    }

    public void SlidesControls() {
        slides.verticalSlidesControls(gamepad2.left_stick_button, gamepad2.right_stick_button);
        slides.horizSlidesControls(gamepad2.right_stick_y > 0.9, gamepad2.right_stick_y < -0.9);
        //slides.PIDF_Vert();
//        slides.PIDF_H();
    }

    public void getTelemetry() {
//        telemetry.addData("rotatePos", arm.rotatePos);
//        telemetry.addData("wristPos", arm.wristPos);
        telemetry.addData("arm rotate pos", arm.rotatePos);
        telemetry.addData("arm target", arm.rightEncoderPos());
        telemetry.addData("vertical slides", slides.slidesPos);
        telemetry.addData("outtake pos", outtake.wristPos);
        telemetry.update();
    }
}