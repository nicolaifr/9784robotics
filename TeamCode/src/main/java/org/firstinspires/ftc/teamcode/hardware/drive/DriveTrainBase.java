package org.firstinspires.ftc.teamcode.hardware.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareBase;
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;

public class DriveTrainBase extends HardwareBase {
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
//    private Follower follower;
//    private final Pose startPose = new Pose(0,0,0);


    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);

        leftBack = ahwMap.get(DcMotor.class, "leftBackWheel");
        rightBack = ahwMap.get(DcMotor.class, "rightBackWheel");
        leftFront = ahwMap.get(DcMotor.class, "leftFrontWheel");
        rightFront = ahwMap.get(DcMotor.class, "rightFrontWheel");

        //reverse left side motors so the thing will actually work
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //resetting encoders and disabling built in PID (run to pos, its buns)
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(ahwMap);
//        follower.setStartingPose(startPose);
//
//        follower.startTeleopDrive();
    }
    public void driveJoystick(double left_stick_y, double left_stick_x, double right_stick_x) {
        double y = left_stick_y; // Remember, Y stick value is reversed
        double x = left_stick_x; // Counteract imperfect strafing
        double rx = right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.8);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
//        if (left_stick_x != 0 || left_stick_y != 0) {
//            follower.setTeleOpMovementVectors(left_stick_y * 0.75, left_stick_x * 0.75, turn, true);
//        } else {
//            follower.setTeleOpMovementVectors(right_stick_y * 0.75, right_stick_x * 0.75, turn, false);
//        }
//        follower.update();
    }
}
