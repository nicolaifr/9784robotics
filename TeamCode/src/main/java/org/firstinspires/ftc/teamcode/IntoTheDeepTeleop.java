package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ArmBase;
import org.firstinspires.ftc.teamcode.hardware.ClawBase;
import org.firstinspires.ftc.teamcode.hardware.DriveTrainBase;

@TeleOp
public class IntoTheDeepTeleop extends OpMode {
    DriveTrainBase drive;
    //arm
    ArmBase arm;
    //claw
    ClawBase claw;
    double clampPos = 0;
    double wristPos = 0;
    @Override
    public void init() {
        drive = new DriveTrainBase();
        arm = new ArmBase();
        claw = new ClawBase();

        drive.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
        claw.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        armControls();
        clawControls();
    }

    public void armControls() {
        arm.armExtendControls(gamepad2.left_bumper, gamepad2.right_bumper, arm.armExtend.getCurrentPosition());
        arm.armRotateControls(gamepad2.right_trigger, gamepad2.left_trigger, arm.armRotate.getCurrentPosition());
    }
    public void clawControls() {
        claw.clawClamp(gamepad2.right_stick_y, clampPos);
        claw.clawWrist(gamepad2.left_stick_x, wristPos);
    }
}