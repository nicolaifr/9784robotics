package opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
public class StraightLine extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(6.25, 60, Math.toRadians(180));
    private final Pose scorePreload = new Pose(30, 65, Math.toRadians(180));

    private final Pose grabOffWall1 = new Pose(16, 24, Math.toRadians(0));
    private final Pose grabOffWall2 = new Pose(11, 24, Math.toRadians(0));
    private final Pose prepForScoring = new Pose(17, 66, Math.toRadians(180));
    private final Pose prepForScoringCP = new Pose(24, 48, Math.toRadians(180));
    private final Pose score2nd = new Pose(30, 66, Math.toRadians(180));
    private final Pose park = new Pose(0, 0, Math.toRadians(180));
    private PathChain testPath, grabOffWall1Path, grabOffWall2Path, prepForScoringPath, score2ndPath, parkPath;

    public ArmBase arm;
    public OutTake claw;

    public SlidesBase slides;

    public void buildPaths() {
        testPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreload)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreload.getHeading())
                .build();
        grabOffWall1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePreload), new Point(grabOffWall1)))
                .setLinearHeadingInterpolation(scorePreload.getHeading(), grabOffWall1.getHeading())
                .build();
        grabOffWall2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabOffWall1), new Point(grabOffWall2)))
                .setLinearHeadingInterpolation(grabOffWall1.getHeading(), grabOffWall2.getHeading())
                .build();
        prepForScoringPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grabOffWall2), new Point(prepForScoringCP), new Point(prepForScoring)))
                .setLinearHeadingInterpolation(grabOffWall2.getHeading(), prepForScoring.getHeading())
                .build();
        score2ndPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepForScoring), new Point(score2nd)))
                .setLinearHeadingInterpolation(prepForScoring.getHeading(), score2nd.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2nd), new Point(park)))
                .setLinearHeadingInterpolation(score2nd.getHeading(), park.getHeading())
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
                follower.followPath(testPath, true);
                arm.setRotateTarget(2675);
                actionTimer.resetTimer();
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    arm.setRotateTarget(2400);
                }
                setPathState(1);
                break;
            case 1: // go near parking for 2nd spec
                if (!follower.isBusy()) {
                    follower.followPath(grabOffWall1Path, true);
                    if (actionTimer.getElapsedTimeSeconds() >= 0.6) {
                        claw.openClamp();
                        arm.setRotateTarget(2450);
                    }
                    if (actionTimer.getElapsedTimeSeconds() >= 2) {
                        arm.setRotateTarget(1200);

                        actionTimer.resetTimer();
                        setPathState(3);
                    }
                }
                break;
            case 3: // shoves robot into the wall to get 2nd spec
                if (!follower.isBusy()) {
                    follower.followPath(grabOffWall2Path, true);
                    claw.openClamp();
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4: // close clamp after some time
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() >= 3){
                        claw.closeClamp();
                    }
                    if (actionTimer.getElapsedTimeSeconds() >= 4.7) {
                        setPathState(5);
                    }
                }
                break;
            case 5: // go to spot before directly to rod spot to rotate robot 180 deg
                if (!follower.isBusy()) {
                    follower.followPath(prepForScoringPath, true);
                    arm.setRotateTarget(2675);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6: // go directly to rod spot
                if (!follower.isBusy()) {
                    follower.followPath(score2ndPath, true);
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                        arm.setRotateTarget(2400); // CHANGE: made arm slam down after it gets there [consider increasing time]
                        claw.openClamp();
                        setPathState(7);
                    }
                }
                break;
            case 7: //
                if (!follower.isBusy()) {
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
