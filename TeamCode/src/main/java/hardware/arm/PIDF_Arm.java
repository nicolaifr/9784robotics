package hardware.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Config
public class PIDF_Arm extends OpMode {

    private PIDController rotateController;
    private PIDController wristController;
    //P,I,D in the PID controller watch KookyBotz Video for more info
    public static double pR = 0.007, iR = 0, dR = 0.0001;
    public static double pE = 0.008, iE = 0, dE = 0.0001;
    //feedforward
    public static double f = 0.15;
    //arm target position
    public static int rotateTarget = 0;
    public static int wristTarget = 0;
    //how many ticks in degree USING REV THROUGH BORE ENCODER
    private final double ticks_in_degree = ((double) 8192) /360;
    private CRServo armRotateLeft;
    private CRServo armRotateRight;
    public DcMotorEx armRotateLeftEncoder;
    public DcMotorEx armRotateRightEncoder;
//hi guys !!!!
    @Override
    public void init() {
        //initialization code when "INIT" is pressed
        rotateController = new PIDController(pR, iR, dR);
        wristController = new PIDController(pE, iE, dE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armRotateLeft = hardwareMap.get(CRServo.class, "ArmLeft");
        armRotateRight = hardwareMap.get(CRServo.class, "ArmRight");

        armRotateLeftEncoder = hardwareMap.get(DcMotorEx.class, "ArmLeftEncoder");
        armRotateRightEncoder = hardwareMap.get(DcMotorEx.class, "ArmRightEncoder");
        armRotateLeftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRotateRightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        //loop code when "PLAY/Triangle" is hit loops over again while opmode is active
        rotateController.setPID(pR, iR, dR);
        wristController.setPID(pE, iE, dE);
        int rotatePos = (armRotateLeftEncoder.getCurrentPosition() + armRotateRightEncoder.getCurrentPosition()) / 2;
        //PID MATH
        double rotatePID = rotateController.calculate(rotatePos, rotateTarget);
        //feedforward math
        double rotateFF = Math.cos(Math.toRadians(rotateTarget/ticks_in_degree)) * f;
        //power calculated
        double rotatePower = rotatePID + rotateFF;

        int wristPos = armRotateLeftEncoder.getCurrentPosition() - armRotateRightEncoder.getCurrentPosition();
        //PID MATH
        double wristPID = wristController.calculate(wristPos, wristTarget);
        //feedforward math
        double wristFF = Math.cos(Math.toRadians(wristTarget/ticks_in_degree)) * f;
        //power calculated
        double wristPower = wristPID + wristFF;

        double leftPower = Range.clip(rotatePower + wristPower, -1, 1);
        double rightPower = Range.clip(rotatePower - wristPower, -1, 1);
        //setting motor power after all those calculations
        armRotateLeft.setPower(leftPower);
        armRotateRight.setPower(rightPower);
        //telemetry for tuning
        telemetry.addData("rotate Pos", rotatePos);
        telemetry.addData("rotate target", rotateTarget);
        telemetry.addData("wrist Pos", wristPos);
        telemetry.addData("wrist target", wristTarget);
        telemetry.update();
    }
}
