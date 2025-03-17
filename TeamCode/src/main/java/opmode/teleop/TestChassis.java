package opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import hardware.arm.ArmBase;
import hardware.claw.Intake;
import hardware.drive.DriveTrainBase;
import hardware.claw.OutTake;
import hardware.slides.SlidesBase;


@TeleOp
public class TestChassis extends OpMode {
    DriveTrainBase drive;


    int clawState;
    @Override
    public void init() {

        drive = new DriveTrainBase();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {

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
//        outtake.wrist(gamepad2.b, gamepad2.a);
//        arm.arm(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.right_bumper, gamepad2.left_bumper);
    }

    public void IntakeControls() {

    }

    public void SlidesControls() {

        //slides.PIDF_Vert();
//        slides.PIDF_H();
    }

    public void getTelemetry() {
//        telemetry.addData("rotatePos", arm.rotatePos);
//        telemetry.addData("wristPos", arm.wristPos);
        telemetry.update();
    }
}