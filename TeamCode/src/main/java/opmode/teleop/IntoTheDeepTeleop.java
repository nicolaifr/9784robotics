package opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import hardware.drive.DriveTrainBase;


@TeleOp
public class IntoTheDeepTeleop extends OpMode {
    DriveTrainBase drive;
    OutTakeBase outtake;

    int clawState;
    @Override
    public void init() {
        drive = new DriveTrainBase();
        outtake = new OutTakeBase();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap, telemetry);
        outtake.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        clawControls();
        getTelemetry();
    }
    public void clawControls() {
    }
    public void OutTakeControls(){
        outtake.clawClamp(gamepad2.left_stick_button);
        outtake.diffyArm(gamepad2.left_stick_y);
        outtake.clawWrist(gamepad2.left_stick_x);
    }

    public void getTelemetry() {
        telemetry.update();
    }
}