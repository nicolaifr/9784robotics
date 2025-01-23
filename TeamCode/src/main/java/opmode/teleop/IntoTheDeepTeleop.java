package opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import hardware.arm.ArmBase;
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
    public void loop() {
        drive.driveJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        getTelemetry();
        OutTakeControls();
        IntakeControls();
        SlidesControls();
    }
    public void OutTakeControls(){
        outtake.clamp(gamepad2.dpad_up, gamepad2.dpad_down);
        outtake.wrist(gamepad2.right_stick_x);
        arm.arm(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.right_bumper, gamepad2.left_bumper);
    }

    public void IntakeControls() {
        intake.clawWrist(gamepad2.b, gamepad2.x);
        intake.clawClamp(gamepad2.a);
//        intake.miniPivot(gamepad1.right_trigger, gamepad1.left_trigger);
        intake.intakeDown(gamepad2.dpad_right, gamepad2.dpad_left);
        intake.PIDF_Pivot();
    }

    public void SlidesControls() {
//        slides.verticalSlidesControls(gamepad2.right_stick_button, gamepad2.left_stick_button);
        slides.horizSlidesControls(gamepad2.right_stick_button, gamepad2.left_stick_button);
        //slides.PIDF_Vert();
//        slides.PIDF_H();
    }

    public void getTelemetry() {
        telemetry.addData("target", intake.monsterPivot.getTargetPosition());
        telemetry.addData("curretn", intake.monsterPivot.getCurrentPosition());
        telemetry.addData("horiz pos", slides.horizSlides.getCurrentPosition());
        telemetry.addData("horiz power", slides.horizSlides.getPower());
        telemetry.update();
    }
}