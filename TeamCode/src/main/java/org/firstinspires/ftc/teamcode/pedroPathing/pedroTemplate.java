package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class pedroTemplate extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD
    }
    PathState pathState;

    private final Pose startPose = new Pose(24,120,Math.toRadians(145)); //template numbers
    private final Pose shootPos = new Pose(48,96,135); //template numbers

    private PathChain reverseTest;

    public void buildPaths(){
        reverseTest = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(33,110), shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(reverseTest, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("GOON");
                }
                break;
            default:
                telemetry.addLine("No state");
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //Add any other init methods like intake, flywheel, etc
        buildPaths();
        follower.setPose(startPose);

    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();
    }
}
