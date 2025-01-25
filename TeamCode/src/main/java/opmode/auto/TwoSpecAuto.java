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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

//{"startPoint":{"x":6.25,"y":65.25,"heading":"constant","degrees":0},
// "lines":[{"endPoint":{"x":40,"y":65.25,"heading":"constant","reverse":true,"degrees":180},
// {"endPoint":{"x":12,"y":24,"heading":"linear","reverse":false,"degrees":0,"startDeg":180,"endDeg":0},
// "endPoint":{"x":40,"y":65.25,"heading":"linear","reverse":false,"startDeg":0,"endDeg":180},
// {"endPoint":{"x":8,"y":8,"heading":"constant","reverse":false,"degrees":180},
@Autonomous
public class TwoSpecAuto extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(7, 62, Math.toRadians(180));
    private final Pose scorePreload = new Pose(33, 62, Math.toRadians(180));
    private final Pose pickUpHumanSpecimen = new Pose(12, 24, Math.toRadians(0));
    private final Pose scoreHumanSpecimen = new Pose(33, 68, Math.toRadians(180));
    private final Pose park = new Pose(8, 8, Math.toRadians(180));
    private Path scorePreloadPath, parkPath;
    private PathChain pickUpHumanSpecimenPath, scoreHumanSpecimenPath;

    public void buildPaths() {
        scorePreloadPath = new Path(new BezierLine(new Point(startPose), new Point(scorePreload)));
        scorePreloadPath.setConstantHeadingInterpolation(startPose.getHeading());

        pickUpHumanSpecimenPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePreload), new Point(pickUpHumanSpecimen)))
                .setLinearHeadingInterpolation(scorePreload.getHeading(), pickUpHumanSpecimen.getHeading())
                .build();

        scoreHumanSpecimenPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUpHumanSpecimen), new Point(scoreHumanSpecimen)))
                .setLinearHeadingInterpolation(pickUpHumanSpecimen.getHeading(), scoreHumanSpecimen.getHeading())
                .build();

        parkPath = new Path(new BezierLine(new Point(scoreHumanSpecimen), new Point(park)));
        parkPath.setConstantHeadingInterpolation(park.getHeading());
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // score preload
                follower.followPath(scorePreloadPath);
                setPathState(1);
                break;

            case 1: // pickup human specimen
                if (!follower.isBusy()) {
                    follower.followPath(pickUpHumanSpecimenPath, false);
                    setPathState(2);
                }
                break;

            case 2: // score human specimen
                if (!follower.isBusy()) {
                    follower.followPath(scoreHumanSpecimenPath, false);
                    setPathState(3);
                }
                break;

            case 3: // park
                if (!follower.isBusy()) {
                    follower.followPath(parkPath);
                    setPathState(4);
                }
                break;

            case 4: // end auto
                if (!follower.isBusy()) {
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop(){
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("position", follower.getPose().toString());
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
