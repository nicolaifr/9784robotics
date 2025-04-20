package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.claw.IntakeClaw;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveTrainBase;

import org.firstinspires.ftc.teamcode.hardware.arm.PivotArm;
import org.firstinspires.ftc.teamcode.hardware.claw.Intake;



@TeleOp
public class TestChassis extends OpMode {
    DriveTrainBase drive;
    PivotArm arm;
    IntakeClaw intake;


    int clawState;
    @Override
    public void init() {

        drive = new DriveTrainBase();
        arm = new PivotArm();
        intake = new IntakeClaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        ArmControl();
        getTelemetry();
        OutTakeControls();
        IntakeControls();
        SlidesControls();
    }
    public void OutTakeControls(){
//        outtake.wrist(gamepad2.b, gamepad2.a);
//        arm.arm(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.right_bumper, gamepad2.left_bumper);
    }

    public void IntakeControls() {
        intake.intakeControl(gamepad2.a, gamepad2.b);
        intake.swivelControl(gamepad2.dpad_up, gamepad2.dpad_down);
        intake.wristControl(gamepad2.dpad_left, gamepad2.dpad_right);
    }

    public void SlidesControls() {

        //slides.PIDF_Vert();
//        slides.PIDF_H();
    }

    public void ArmControl() {
        arm.rotateArm(gamepad1.right_trigger, gamepad1.left_trigger);
        arm.extendArm(gamepad1.right_bumper, gamepad1.left_bumper);
    }

    public void getTelemetry() {
//        telemetry.addData("rotatePos", arm.rotatePos);
//        telemetry.addData("wristPos", arm.wristPos);
        telemetry.update();
    }
}