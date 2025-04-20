package org.firstinspires.ftc.teamcode.hardware.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class PIDF_ArmLeftRight extends OpMode {

    private PIDController leftController;
    private PIDController rightController;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double pR = 0.0007, iR = 0, dR = 0.00002;
    //feedforward
    public static double f = -0.05;
    //arm target position
    public static int rotateTarget = 0;
    public static int wristTarget = 0;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = ((double) 8192) / 360;
    private CRServo armRotateLeft;
    private CRServo armRotateRight;
    public DcMotorEx armRotateLeftEncoder;
    public DcMotorEx armRotateRightEncoder;
    //hi guys !!!!
    @Override
    public void init() {
        //initialization code when "INIT" is pressed

        leftController = new PIDController(pR, iR, dR);
        rightController = new PIDController(pR, iR, dR);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armRotateLeft = hardwareMap.get(CRServo.class, "ArmLeft");
        armRotateRight = hardwareMap.get(CRServo.class, "ArmRight");


        armRotateLeftEncoder = hardwareMap.get(DcMotorEx.class, "leftFrontWheel");
        armRotateRightEncoder = hardwareMap.get(DcMotorEx.class, "rightFrontWheel");

        armRotateLeftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRotateRightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRotateLeftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armRotateRightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        armRotateLeftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        leftController.setPID(pR, iR, dR);
        rightController.setPID(pR, iR, dR);

        int leftPos = -armRotateLeftEncoder.getCurrentPosition();
        double leftTarget = rotateTarget - (double) wristTarget /2;
        //PID MATH
        double leftPID = leftController.calculate(leftPos, leftTarget);
        //feedforward math
        double leftFF = Math.cos(Math.toRadians(leftTarget/ticks_in_degree)) * f;
        //power calculated
        double leftPower = leftPID + leftFF;

        int rightPos = -armRotateRightEncoder.getCurrentPosition();
        double rightTarget = rotateTarget + (double) wristTarget /2;
        //PID MATH
        double rightPID = rightController.calculate(rightPos, rightTarget);
        //feedforward math
        double rightFF = Math.cos(Math.toRadians(rightTarget/ticks_in_degree)) * f;
        //power calculated
        double rightPower = rightPID + rightFF;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);
        //setting motor power after all those calculations
        armRotateLeft.setPower(leftPower);
        armRotateRight.setPower(rightPower);
        //telemetry for tuning

        int rotatePos = (leftPos+rightPos)/2;
        int wristPos = (leftPos-rightPos);

        telemetry.addData("right pos", rightPos);
        telemetry.addData("left pos", leftPos);
        telemetry.addData("right target", rightTarget);
        telemetry.addData("left target", leftTarget);
        telemetry.addData("right power", rightPower);
        telemetry.addData("left power", leftPower);
        telemetry.addData("rotate Pos", rotatePos);
        telemetry.addData("rotate target", rotateTarget);
        telemetry.addData("wrist pos", wristPos);
        telemetry.addData("wrist target", wristTarget);
        telemetry.update();
    }
}
