package opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import hardware.drive.DriveTrainBase;
import hardware.claw.OutTake;


@TeleOp
public class IntoTheDeepTeleop extends OpMode {
    DriveTrainBase drive;
    OutTake outtake;

    int clawState;
    @Override
    public void init() {
        drive = new DriveTrainBase();
        outtake = new OutTake();

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
        outtake.clamp(gamepad2.right_bumper, gamepad2.left_bumper);
        outtake.arm(gamepad2.a, gamepad2.b);
        outtake.wrist(gamepad2.right_trigger, gamepad2.left_trigger);
    }

    public void getTelemetry() {
        telemetry.update();
    }
}