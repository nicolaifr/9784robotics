package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.arm.ArmBase;
import org.firstinspires.ftc.teamcode.hardware.claw.ClawBase;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveTrainBase;

@TeleOp
public class IntoTheDeepTeleop extends OpMode {
    DriveTrainBase drive;
    //arm
    ArmBase arm;
    //claw
    ClawBase claw;

    int clawState;
    @Override
    public void init() {
        drive = new DriveTrainBase();
        arm = new ArmBase();
        claw = new ClawBase();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
        claw.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        armControls();
        clawControls();
        getTelemetry();
    }

    public void armControls() {
        arm.armExtendControls(gamepad2.left_bumper, gamepad2.right_bumper);
        arm.armRotateControls(gamepad2.right_trigger, gamepad2.left_trigger);
    }
    public void clawControls() {
        claw.clawClamp(gamepad2.right_stick_button);
        claw.clawWrist(gamepad2.right_stick_x);
    }

    public void getTelemetry() {
        telemetry.addData("armRotatePos", arm.armRotate.getCurrentPosition());
        telemetry.addData("armExtendPos", arm.armExtend.getCurrentPosition());
        telemetry.addData("clampPos", claw.clampPos);
        telemetry.addData("wristPos", claw.wristPos);
        telemetry.update();
    }
}