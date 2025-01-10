package opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import hardware.claw.Intake;
import hardware.claw.PIDF_Pivot;
import hardware.drive.DriveTrainBase;
import hardware.claw.OutTake;
import hardware.slides.SlidesBase;


@TeleOp
public class IntoTheDeepTeleop extends OpMode {
    DriveTrainBase drive;
    OutTake outtake;
    SlidesBase slides;
    Intake intake;

    int clawState;
    @Override
    public void init() {
        drive = new DriveTrainBase();
        outtake = new OutTake();
        intake = new Intake();
        slides = new SlidesBase();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap, telemetry);
        outtake.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        slides.init(hardwareMap, telemetry);
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
        outtake.clamp(gamepad2.right_bumper, gamepad2.left_bumper);
        outtake.arm(gamepad2.a, gamepad2.b);
        outtake.wrist(gamepad2.right_trigger, gamepad2.left_trigger);
    }

    public void IntakeControls() {
        intake.clawWrist(gamepad1.b, gamepad1.x);
        intake.miniPivot(gamepad1.right_trigger, gamepad1.left_trigger);
        intake.clawClamp(gamepad1.a);
        intake.intakeDown(gamepad1.right_bumper, gamepad1.left_bumper);
        intake.PIDF_Pivot();
    }

    public void SlidesControls() {
        slides.verticalSlidesControls(gamepad2.right_stick_button, gamepad2.left_stick_button);
        slides.horizSlidesControls(gamepad1.left_stick_button, gamepad1.right_stick_button);
        slides.PIDF_Vert();
    }

    public void getTelemetry() {
        telemetry.update();
    }
}