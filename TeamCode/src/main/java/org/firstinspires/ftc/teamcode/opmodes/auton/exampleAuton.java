package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LancersRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "exampleAuton")
public class exampleAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // --- STATE MACHINE ---
    public enum PathState {
        STARTPOS_TO_SHOOTPOS,
        SHOOT,
        COLLECT,
        SHOOTPOS_TO_LEAVEPOS,
        DONE
    }

    private PathState pathState;
    private boolean justEntered = true;

    // Cycle index for collecting and shooting
    private int cycleIndex = 0;

    // Relevant Poses
    private final Pose startPose = new Pose(20, 122, Math.toRadians(138));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(138));
    private final Pose leavePose = new Pose(32, 80, Math.toRadians(138));

    // Paths
    private PathChain startPosToShootPos, shootPosToLeavePos;
    private PathChain collectFirstBalls, collectSecondBalls, collectThirdBalls;

    // Timings
    private static final double SHOOT_SECONDS = 1.0; // TODO: tune later

    public void buildPaths() {
        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootPosToLeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();


        // Collect 1
        collectFirstBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(68.291, 82.036),
                        new Pose(54.545, 82.018),
                        new Pose(40.800, 82.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(40.800, 82.000),
                        new Pose(-10.000, 83.345),
                        new Pose(15.000, 83.700),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Collect 2
        collectSecondBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(77.891, 61.527),
                        new Pose(49.346, 59.764),
                        new Pose(20.800, 58.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(20.800, 58.000),
                        new Pose(49.527, 71.127),
                        new Pose(49.763, 83.564),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Collect 3
        collectThirdBalls = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(83.127, 25.964),
                        new Pose(51.964, 29.982),
                        new Pose(20.800, 34.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(20.800, 34.000),
                        new Pose(49.527, 71.127),
                        new Pose(49.763, 83.564),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    // Changes path state
    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        justEntered = true;
    }

    // Chooses the correct shooting and collecting path for the current cycle
    private PathChain getCollectPathForCycle(int cycleIndex) {
        switch (cycleIndex) {
            case 0: return collectFirstBalls;
            case 1: return collectSecondBalls;
            default: return collectThirdBalls;
        }
    }

    private void statePathUpdate() {
        switch (pathState) {

            case STARTPOS_TO_SHOOTPOS: {
                if (justEntered) {
                    follower.followPath(startPosToShootPos, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    // arrived at shoot pose for the first time
                    cycleIndex = 0;
                    setPathState(PathState.SHOOT);
                }
                break;
            }

            case SHOOT: {
                if (justEntered) {
                    // TODO: start flywheel / feeder here
                    justEntered = false;
                }

                // TODO: keep shooter running here if needed
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
                    // TODO: stop feeder if needed
                    if (cycleIndex < 3) {
                        setPathState(PathState.COLLECT); // Goes onto the next collect cycle
                    } else {
                        setPathState(PathState.SHOOTPOS_TO_LEAVEPOS); // Leaves
                    }
                }
                break;
            }

            case COLLECT: {
                if (justEntered) {
                    // TODO: start intake here
                    follower.followPath(getCollectPathForCycle(cycleIndex), true);
                    justEntered = false;
                }

                if (!follower.isBusy()) {
                    // TODO: stop intake if needed
                    cycleIndex++; // Completed one cycle, going onto the next one
                    setPathState(PathState.SHOOT);
                }
                break;
            }

            case SHOOTPOS_TO_LEAVEPOS: {
                if (justEntered) {
                    follower.followPath(shootPosToLeavePos, true);
                    justEntered = false;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;
            }

            case DONE:
            default: {
                telemetry.addLine("Auton Complete");
                break;
            }
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        setPathState(PathState.STARTPOS_TO_SHOOTPOS);

        LancersRobot robot = new LancersRobot(hardwareMap, telemetry);
        // TODO: initialize intake/outtake
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        justEntered = true;
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Cycle Index", cycleIndex);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
    }
}
