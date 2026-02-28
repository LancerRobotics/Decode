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

@Autonomous(name = "Shoot3Far")
public class Shoot3Far extends OpMode {
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
        SHOOT,
        PARK
    }

    private PathState pathState;
    private boolean justEntered = true;



    // Relevant Poses
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose leavePose = new Pose(0, 18, Math.toRadians(0));

    // Paths
    private PathChain startPosToLeavePos;
    // Timings
    private static final double SHOOT_SECONDS = 1.0; // TODO: tune later

    public void buildPaths() {

        startPosToLeavePos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leavePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leavePose.getHeading())
                .build();
    }

    // Changes path state
    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        justEntered = true;
    }


    private void statePathUpdate() {
        switch (pathState) {

            case SHOOT: {

                if (justEntered) {
                    justEntered = false;
                    robot.setServoPosition(0.8);
                    robot.setIntake(1);
                    robot.setOuttakePIDF(robot.farShootingPIDF);
                    robot.setOuttakeVelocity(1240);
                }

                if (pathTimer.getElapsedTimeSeconds() >= 10) {
                    robot.setOuttakeTwoPower(0.9);
                }

                // start intake and outtake for the rest of the auton


                if (pathTimer.getElapsedTimeSeconds() >= 10+5) {
                    if (!follower.isBusy()) {
                        // arrived at shoot pose for the first time
                        setPathState(PathState.PARK);
                    }
                }
                break;
            }

            case PARK: {
                if (justEntered) {
                    follower.followPath(startPosToLeavePos, true);
                    justEntered = false;
                }
                break;
            }

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

        setPathState(PathState.SHOOT);

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
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
    }
}