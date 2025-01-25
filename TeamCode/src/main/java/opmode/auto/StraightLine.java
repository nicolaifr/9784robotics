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
public class StraightLine extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(6.25, 60, Math.toRadians(180));
    private final Pose park = new Pose(33, 60, Math.toRadians(0));
    private Path testPath;

    public void buildPaths() {

        testPath = new Path(new BezierLine(new Point(startPose), new Point(park)));
        testPath.setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading());

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // score preload
                follower.followPath(testPath, false);
                setPathState(1);
                break;
            case 1:
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
