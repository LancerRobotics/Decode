package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.LancersBotConfig;
import org.firstinspires.ftc.teamcode.LancersRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.LimelightWrapper;

@Autonomous(name = "Blue12")
public class Blue12 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LancersRobot robot;

    private LimelightWrapper limelightWrapper;

    // Drivetrain motors
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    // --- STATE MACHINE ---
    public enum PathState {
        STARTPOS_TO_SHOOTPOS,
        SHOOT,
        COLLECT,
        ALIGN_TO_BALL,
        RETURN_TO_SHOOT,
        SHOOTPOS_TO_LEAVEPOS,
        DONE
    }

    private PathState pathState;
    private boolean justEntered = true;

    // Cycle index for collecting and shooting
    private int cycleIndex = 0;

    // Relevant Poses
    private final Pose startPose = new Pose(23.978, 122.736, Math.toRadians(138.630));
    private final Pose shootPose = new Pose(48+4, 96+4, Math.toRadians(130));
    private final Pose leavePose = new Pose(36.000, 72.000, Math.toRadians(90));

    // Paths
    private PathChain startPosToShootPos, shootPosToLeavePos;
    private PathChain collectFirstBalls, collectSecondBalls, collectThirdBalls;
    private PathChain collectFirstBalls_approach, collectSecondBalls_approach, collectThirdBalls_approach;
    private PathChain collectFirstBalls_return, collectSecondBalls_return, collectThirdBalls_return;

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


        // Collect 1 - split into approach and return
        collectFirstBalls_approach = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose, new Pose(48.000,84.000)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .build();

        collectFirstBalls_return = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(23.160, 84.462),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        // Keep the full path for backwards compatibility
        collectFirstBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose, new Pose(48.000,84.000)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(48.000,84.000), new Pose(23.160, 84.462)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(23.160, 84.462),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();


        // Collect 2 - split into approach and return
        collectSecondBalls_approach = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose, new Pose(48.000,60.000)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .build();

        collectSecondBalls_return = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(19.811, 57.915),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        // Keep the full path for backwards compatibility
        collectSecondBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose, new Pose(48.000,60.000)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(48.000,60.000), new Pose(19.811, 57.915)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(19.811, 57.915),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();


        // Collect 3 - split into approach and return
        collectThirdBalls_approach = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose, new Pose(48.000,36.000)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .build();

        collectThirdBalls_return = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(19.160, 35.446),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        // Keep the full path for backwards compatibility
        collectThirdBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose, new Pose(48.000,36.000)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(48.000,36.000), new Pose(19.160, 35.446)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(19.160, 35.446),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
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

    private PathChain getCollectApproachPathForCycle(int cycleIndex) {
        switch (cycleIndex) {
            case 0: return collectFirstBalls_approach;
            case 1: return collectSecondBalls_approach;
            default: return collectThirdBalls_approach;
        }
    }

    private PathChain getCollectReturnPathForCycle(int cycleIndex) {
        switch (cycleIndex) {
            case 0: return collectFirstBalls_return;
            case 1: return collectSecondBalls_return;
            default: return collectThirdBalls_return;
        }
    }


    private void statePathUpdate() {
        switch (pathState) {

            case STARTPOS_TO_SHOOTPOS: {

                // start intake and outtake for the rest of the auton
                robot.setOuttakeVelocity(940);
                robot.setIntake(0.9);

                robot.setServoPosition(1);

                if (justEntered) {
                    follower.followPath(startPosToShootPos, true);
                    justEntered = false;
                }

                if (pathTimer.getElapsedTimeSeconds() >= 3) {
                    if (!follower.isBusy()) {
                        // arrived at shoot pose for the first time
                        cycleIndex = 0;
                        setPathState(PathState.SHOOT);
                    }
                }
                break;
            }

            case SHOOT: {
                if (justEntered) {
                    justEntered = false;
                }

                if (pathTimer.getElapsedTimeSeconds() >= ((cycleIndex==0) ? 2 : 0)) {
                    robot.setOuttakeTwoPower(1);
                }
                if (pathTimer.getElapsedTimeSeconds() >= ((cycleIndex==0) ? 2 : 0) + SHOOT_SECONDS) {
                    robot.setOuttakeTwoPower(0);
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
                    follower.followPath(getCollectApproachPathForCycle(cycleIndex), true);
                    justEntered = false;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.ALIGN_TO_BALL);
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

            case ALIGN_TO_BALL: {
                if (justEntered) {
                    limelightWrapper.switchPipeline(0);
                    justEntered = false;
                }

                double ty = limelightWrapper.getBallTy();

                // CONSTANTS
                double kp = 0.03; // AKA speed of lateral strafe
                double forwardPower = 0.3;

                if (follower.getPose().getX() > 24) {
                    follower.breakFollowing();

                    double strafePower = kp * ty;
                    strafePower = Math.max(-0.4, Math.min(0.4, strafePower));

                    leftFront.setPower(forwardPower - strafePower);
                    leftRear.setPower(forwardPower + strafePower);
                    rightFront.setPower(forwardPower + strafePower);
                    rightRear.setPower(forwardPower - strafePower);
                } else {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);

                    setPathState(PathState.RETURN_TO_SHOOT);
                }
                break;
            }

            case RETURN_TO_SHOOT: {
                if (justEntered) {
                    follower.followPath(getCollectReturnPathForCycle(cycleIndex), true);
                    cycleIndex++;
                    justEntered = false;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT);
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
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftRear  = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightRear  = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);

        limelightWrapper = new LimelightWrapper(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        robot = new LancersRobot(hardwareMap, telemetry, true, false, true);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        setPathState(PathState.STARTPOS_TO_SHOOTPOS);

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