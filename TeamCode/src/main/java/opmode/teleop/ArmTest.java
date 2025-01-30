package opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import hardware.arm.ArmBase;
import hardware.claw.Intake;
import hardware.claw.OutTake;
import hardware.drive.DriveTrainBase;
import hardware.slides.SlidesBase;



public class ArmTest extends OpMode {
    ArmBase arm;
    Intake intake;

    int clawState;
    @Override
    public void init() {

        arm = new ArmBase();
        intake = new Intake();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        getTelemetry();
        IntakeControls();
    }
    public void OutTakeControls(){

        arm.arm(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.right_bumper, gamepad2.left_bumper);
    }

    public void IntakeControls() {
        intake.clawClamp(gamepad2.a);
        intake.clawWrist(gamepad2.dpad_up, gamepad2.dpad_down);
    }

    public void SlidesControls() {
//        slides.verticalSlidesControls(gamepad2.right_stick_button, gamepad2.left_stick_button);
        //slides.PIDF_Vert();
//        slides.PIDF_H();
    }

    public void getTelemetry() {
        telemetry.addData("left position", arm.armRotateLeftEncoder.getCurrentPosition());
        telemetry.addData("armPos", arm.rotatePos);
        telemetry.addData("right position", arm.armRotateRightEncoder.getCurrentPosition());
        telemetry.update();
    }
}