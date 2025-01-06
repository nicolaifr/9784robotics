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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class FourSpecAuto extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(6.25, 65.75, Math.toRadians(0));
    private final Pose scorePreload = new Pose(41.75, 65.75, Math.toRadians(0));
    private final Pose goToSample1 = new Pose(48, 36, Math.toRadians(0));
    private final Pose goToSample1ControlPt = new Pose(27, 45, Math.toRadians(0));
    private final Pose pushSample1 = new Pose(12, 45, Math.toRadians(0));
    private final Pose pushSample1ControlPt = new Pose(100, 24, Math.toRadians(0));
    private final Pose goToSample2 = new Pose(60, 20, Math.toRadians(0));
    private final Pose pushSample2 = new Pose(12, 12, Math.toRadians(0));
    private final Pose pushSample2ControlPt = new Pose(56, 8, Math.toRadians(0));
    private final Pose goToPickupPrep = new Pose(18.5, 24, Math.toRadians(0)); //used for all specs
    private final Pose goToPickupPrepControlPt = new Pose(24, 18, Math.toRadians(0)); //used once
    private final Pose goToPickup = new Pose(7, 24, Math.toRadians(0)); //used for all specs
    private final Pose score1stSpec = new Pose(41.75, 70, Math.toRadians(180));
    private final Pose score1stSpecControlPt = new Pose(12, 70, Math.toRadians(180));
    private final Pose score2ndSpec = new Pose(41.75, 75, Math.toRadians(180));
    private final Pose score2ndSpecControlPt = new Pose(19, 70, Math.toRadians(180));
    private final Pose score3rdSpec = new Pose(41.75, 80, Math.toRadians(180));
    private final Pose score3rdSpecControlPt = new Pose(19, 75, Math.toRadians(180));
    private final Pose park = new Pose(13, 24, Math.toRadians(0));
    private final Pose parkControlPt = new Pose(19, 75, Math.toRadians(180));

    private Path scorePreload1, parkPath;
    private PathChain goToSample1path, pushSample1path, goToSample2path, pushSample2path,
            goToPickupPreppath, goToPickuppath, score1spec, goToPickupPrep1st, goToPickuppath2, score2spec, goToPickupPrep2nd, goToPickuppath3, score3spec;

    public void buildPaths() {
        scorePreload1 = new Path(new BezierLine(new Point(startPose), new Point(scorePreload)));
        scorePreload1.setConstantHeadingInterpolation(startPose.getHeading());

         goToSample1path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreload), new Point(goToSample1ControlPt), new Point(goToSample1)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();

        pushSample1path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(goToSample1), new Point(pushSample1ControlPt), new Point(pushSample1)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        goToSample2path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushSample1), new Point(goToSample2)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        pushSample2path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(goToSample2), new Point(pushSample2ControlPt), new Point(pushSample2)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        goToPickupPreppath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushSample2), new Point(goToPickupPrepControlPt), new Point(goToPickupPrep)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        goToPickuppath = follower.pathBuilder() //used for all the scoring
                .addPath(new BezierLine(new Point(goToPickupPrep), new Point(goToPickup)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        goToPickuppath2 = follower.pathBuilder() //used for all the scoring
                .addPath(new BezierLine(new Point(goToPickupPrep), new Point(goToPickup)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        goToPickuppath3 = follower.pathBuilder() //used for all the scoring
                .addPath(new BezierLine(new Point(goToPickupPrep), new Point(goToPickup)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        score1spec = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(goToPickup), new Point(score1stSpecControlPt), new Point(score1stSpec)))
                .setLinearHeadingInterpolation(goToSample1.getHeading(), score1stSpec.getHeading())
                .build();
        goToPickupPrep1st = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1stSpec), new Point(goToPickupPrep)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        score2spec = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(goToPickup), new Point(score2ndSpecControlPt), new Point(score2ndSpec)))
                .setLinearHeadingInterpolation(goToSample1.getHeading(), score1stSpec.getHeading())
                .build();
        goToPickupPrep2nd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2ndSpec), new Point(goToPickupPrep)))
                .setConstantHeadingInterpolation(goToSample1.getHeading())
                .build();
        score3spec = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(goToPickup), new Point(score3rdSpecControlPt), new Point(score3rdSpec)))
                .setLinearHeadingInterpolation(goToSample1.getHeading(), score1stSpec.getHeading())
                .build();

        parkPath = new Path(new BezierCurve(new Point(score3rdSpec), new Point(parkControlPt), new Point(park)));
        parkPath.setConstantHeadingInterpolation(park.getHeading());
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(scorePreload1);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the 1st sample
                if (follower.getPose().getX() > (scorePreload.getX() - 1) && follower.getPose().getY() > (scorePreload.getY() - 1)) {
                    follower.followPath(goToSample1path, true);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is next to scoring position than push
                if (follower.getPose().getX() > (goToSample1.getX() - 1) && follower.getPose().getY() > (goToSample1.getY() - 1)) {
                    follower.followPath(pushSample1path, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the goes to the sample 2 push positoin
                if (follower.getPose().getX() > (pushSample1.getX() - 1) && follower.getPose().getY() > (pushSample1.getY() - 1)) {
                    follower.followPath(goToSample2path, true);
                    setPathState(4);
                }
                break;

            case 4: // push the 2nd sampleon a spline
                if (follower.getPose().getX() > (goToSample2.getX() - 1) && follower.getPose().getY() > (goToSample2.getY() - 1)) {
                    follower.followPath(pushSample2path, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (pushSample2.getX() - 1) && follower.getPose().getY() > (pushSample2.getY() - 1)) {
                    follower.followPath(goToPickupPreppath, true);
                    setPathState(6);
                }
                break;

            case 6: // Wait until the robot is near the third sample pickup position
                if (follower.getPose().getX() > (goToPickupPrep.getX() - 1) && follower.getPose().getY() > (goToPickupPrep.getY() - 1)) {
                    follower.followPath(goToPickuppath, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (goToPickup.getX() - 1) && follower.getPose().getY() > (goToPickup.getY() - 1)) {
                    follower.followPath(score1spec, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (follower.getPose().getX() > (score1stSpec.getX() - 1) && follower.getPose().getY() > (score1stSpec.getY() - 1)) {
                    follower.followPath(goToPickupPrep1st);
                    setPathState(9);
                }
                break;
            case 9: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (goToPickupPrep.getX() - 1) && follower.getPose().getY() > (goToPickupPrep.getY() - 1)) {
                    follower.followPath(goToPickuppath2, true);
                    setPathState(10);
                }
                break;
            case 10: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (goToPickup.getX() - 1) && follower.getPose().getY() > (goToPickup.getY() - 1)) {
                    follower.followPath(score2spec, true);
                    setPathState(11);
                }
                break;
            case 11: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (score2ndSpec.getX() - 1) && follower.getPose().getY() > (score2ndSpec.getY() - 1)) {
                    follower.followPath(goToPickupPrep2nd, true);
                    setPathState(12);
                }
                break;
            case 12: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (goToPickupPrep.getX() - 1) && follower.getPose().getY() > (goToPickupPrep.getY() - 1)) {
                    follower.followPath(goToPickuppath3, true);
                    setPathState(13);
                }
                break;
            case 13: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (goToPickup.getX() - 1) && follower.getPose().getY() > (goToPickup.getY() - 1)) {
                    follower.followPath(score3spec, true);
                    setPathState(14);
                }
                break;
            case 14: // Wait until the robot returns to the scoring position
                if (follower.getPose().getX() > (score3rdSpec.getX() - 1) && follower.getPose().getY() > (score3rdSpec.getY() - 1)) {
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
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop(){}

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
