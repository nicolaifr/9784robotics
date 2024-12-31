package opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import hardware.claw.ClawBase;
import hardware.drive.DriveTrainBase;


@TeleOp
public class IntoTheDeepTeleop extends OpMode {
    DriveTrainBase drive;
    ClawBase claw;

    int clawState;
    @Override
    public void init() {
        drive = new DriveTrainBase();
        claw = new ClawBase();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap, telemetry);
        claw.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        clawControls();
        getTelemetry();
    }
    public void clawControls() {
        claw.clawClamp(gamepad2.right_stick_button);
        claw.clawWrist(gamepad2.right_stick_x);
    }

    public void getTelemetry() {
        telemetry.update();
    }
}