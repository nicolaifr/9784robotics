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

        clawState = 0;

        drive.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
        claw.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        armControls();
        clawControls();
        telemetry.addData("armRotatePos", arm.armRotate.getCurrentPosition());
        telemetry.addData("armExtendPos", arm.armExtend.getCurrentPosition());
        telemetry.addData("clampPos", claw.clampPos);
        telemetry.addData("wristPos", claw.wristPos);
        telemetry.update();
    }

    public void armControls() {
        arm.armExtendControls(gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.options);
        arm.armRotateControls(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.options);
    }
    public void clawControls() {
//        claw.clawClamp(gamepad2.right_stick_y, gamepad2.options);
        switch (clawState) {
            case 0: // waits for gamepad2.a to be pressed
                if (gamepad2.a) {
                    clawState = 1;
                }
                break;
            case 1: // opens/closes claw when gamepad2.a is released
                if (!gamepad2.a) {
                    if (claw.clampOpen) {
                        claw.closeClaw();
                    } else {
                        claw.openClaw();
                    }
                    clawState = 0;
                }
        }
        claw.clawWrist(gamepad2.left_stick_x, gamepad2.options);
    }
}