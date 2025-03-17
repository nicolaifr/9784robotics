package opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import hardware.arm.ArmBase;
import hardware.claw.OutTake;
import hardware.slides.SlidesBase;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

//{"startPoint":{"x":6.25,"y":65.25,"heading":"constant","degrees":0},
// "lines":[{"endPoint":{"x":40,"y":65.25,"heading":"constant","reverse":true,"degrees":180},
// {"endPoint":{"x":12,"y":24,"heading":"linear","reverse":false,"degrees":0,"startDeg":180,"endDeg":0},
// "endPoint":{"x":40,"y":65.25,"heading":"linear","reverse":false,"startDeg":0,"endDeg":180},
// {"endPoint":{"x":8,"y":8,"heading":"constant","reverse":false,"degrees":180},
@Autonomous
public class parkAuto extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(6.25, 60, Math.toRadians(180));
    private final Pose park = new Pose(0, 0, Math.toRadians(180));
    private PathChain testPath, grabOffWall1Path, grabOffWall2Path, prepForScoringPath, score2ndPath, parkPath;

    public ArmBase arm;
    public OutTake claw;

    public SlidesBase slides;

    public void buildPaths() {
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(park)))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
    }
    // 1: move from auto start to rod base in a straight line
    // 2: from the rod base, move to pre-deposit spot, rotate down to outtake and reopen clamp
    // 3: after deposit, move directly to parking spot with open clamp (alr open), bump to wall to align, and (after some time) close clamp
    // 4: go to the pre-rotate spot from parking spot
    // 5: rotate 180deg, and move to rod base
    public void autonomousPathUpdate() {
        switch (pathState)
        {
            case 0: // score preload
                actionTimer.resetTimer();
                setPathState(1);
                break;
            case 7: //
                if (actionTimer.getElapsedTimeSeconds() > 20) {
                    follower.followPath(parkPath, true);
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        arm = new ArmBase();
        claw = new OutTake();
        slides = new SlidesBase();


        slides.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
        claw.init(hardwareMap, telemetry);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.8);
        buildPaths();
    }

    @Override
    public void init_loop(){
        claw.closeClamp();
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        arm.PIDFrotateTo();
        slides.PIDF_H();
        follower.setMaxPower(0.8);

        claw.wrist.setPosition(0.02);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        arm.PIDFrotateTo();
        slides.PIDF_H();
        follower.setMaxPower(0.8);


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("position", follower.getPose().toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("arm target", arm.getRotateTarget());
        telemetry.addData("arm rotate", arm.rotatePos);
        telemetry.update();
    }
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        setPathState(0);
    }
}
