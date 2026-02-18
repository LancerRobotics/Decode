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

    private final LimelightWrapper limelightWrapper = new LimelightWrapper(hardwareMap);

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
        SHOOTPOS_TO_LEAVEPOS,

        ALIGN_TO_BALL,
        DONE
    }

    private PathState pathState;
    private boolean justEntered = true;

    // Cycle index for collecting and shooting
    private int cycleIndex = 0;

    // Relevant Poses
    private final Pose startPose = new Pose(20, 122, Math.toRadians(136));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(136));
    private final Pose leavePose = new Pose(32, 80, Math.toRadians(136));

    // Paths
    private PathChain startPosToShootPos, shootPosToLeavePos;
    private PathChain collectFirstBalls, collectSecondBalls, collectThirdBalls;

    // Timings
    private static final double SHOOT_SECONDS = 1.5; // TODO: tune later

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
                .addPath(new BezierLine(shootPose, new Pose(48.000, 92.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(48.000, 92.000),
                        new Pose(49.941, 82.414),
                        new Pose(43.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(43.000, 84.000),
                        new Pose(36.000, 84.000)
                )).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(36, 84),
                        shootPose
                )).setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        // Collect 2
        collectSecondBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(48.000, 92.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(48.000, 92.000),
                        new Pose(49.941, 58),
                        new Pose(43.000, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(185))
                .addPath(new BezierLine(
                        new Pose(43.000, 60.000),
                        new Pose(30, 60.000)
                )).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(30, 60),
                        shootPose
                )).setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        // Collect 3
        collectThirdBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(48.000, 48.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                //.addPath(new BezierCurve(
                //        new Pose(48.000, 20.000),
                //        new Pose(49.941, 58.414),
                //        new Pose(43.000, 60.000)
                //))
                //.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(48, 48.000),
                        new Pose(30, 48.000)
                )).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(30, 48),
                        shootPose
                )).setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
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

                // start intake and outtake for the rest of the auton
                robot.setOuttakeVelocity(1500);
                robot.setIntake(1);

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
                    justEntered = false;
                }

                robot.setOuttakeTwoPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
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
                    follower.followPath(getCollectPathForCycle(cycleIndex), true);
                    justEntered = false;
                }

                if (!follower.isBusy()) {
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

            case ALIGN_TO_BALL: {
                double ty = limelightWrapper.getBallTy();

                // CONSTANTS
                double kp = 0.03; // AKA speed of turn
                double tolerance = 1; // the smaller this value is, the more strict we want our accuracy
                double maxAdjustTime = 2.5; // how long do we want the bot to adjust for at most

                if(Math.abs(ty) > tolerance && pathTimer.getElapsedTimeSeconds() < maxAdjustTime) {
                    follower.breakFollowing();

                    double turnPower = kp * ty;
                    turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
                    leftFront.setPower(turnPower);
                    leftRear.setPower(turnPower);
                    rightFront.setPower(-turnPower);
                    rightRear.setPower(-turnPower);
                }
                else {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    setPathState(PathState.COLLECT);
                }
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