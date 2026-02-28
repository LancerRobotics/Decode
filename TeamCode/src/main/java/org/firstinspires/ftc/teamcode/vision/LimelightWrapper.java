package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.LancersRobot;

import java.util.List;
import java.util.Arrays;

public class LimelightWrapper {
    private final Limelight3A limelight;

    // center of an FTC field (in inches) for pedro is (72, 72), for limelight its (0,0)
    private static final double PEDRO_X_OFFSET_IN = 72.0;
    private static final double PEDRO_Y_OFFSET_IN = 72.0;
    private static int tagId;

    public int getTagId() {
        return tagId;
    }

    public LimelightWrapper(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LancersBotConfig.LIMELIGHT);
        limelight.pipelineSwitch(9); // TODO: Change this pipeline to the correct pipeline later
        limelight.start();
    }

    // returns a 2D pose with heading (in radians) if an april tag is found.
    // Uses getRobotPoseFieldSpace() per-fiducial (not getBotpose()) so the pose
    // is tied to a specific known tag, giving a more reliable field-space position.
    public Pose getBotPose() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        LLResultTypes.FiducialResult validFr = null;

        tagId = 0;

        for (LLResultTypes.FiducialResult fr : tags) {
            int id = fr.getFiducialId();
            if (id == 20 || id == 24) {
                validFr = fr;
                tagId = id;
                break;
            }
        }

        if (validFr == null) return null;

        Pose3D cameraPose = validFr.getRobotPoseFieldSpace();
        if (cameraPose == null) return null;

        // convert meters to inches and shift field origin from center to corner (+72)
        double xIn = cameraPose.getPosition().x * 39.37 + PEDRO_X_OFFSET_IN;
        double yIn = cameraPose.getPosition().y * 39.37 + PEDRO_Y_OFFSET_IN;
        double headingRad = Math.toRadians(cameraPose.getOrientation().getYaw());

        return new Pose(xIn, yIn, headingRad);
    }

    public double getTy() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        boolean validTagSeen = false;

        tagId = 0;

        for (LLResultTypes.FiducialResult fr : tags) {
            int id = fr.getFiducialId();
            if (id == 20 || id == 24) {
                validTagSeen = true;
                tagId = id;
                break;
            }
        }

        if (!validTagSeen) return 0;

        return result.getTy();
    }

    double tagX;
    double tagY;

    public double getDistanceToTag(boolean redMode) {
        Pose pose = this.getBotPose();

        if (pose == null) {
            return 0.0;
        }

        // x and y are switched since the robot is mounted perpendicularly
        double botY = -pose.getX();
        double botX = -pose.getY();

        //replace these values with the coordinates of the actual apriltags respectively
        if (!redMode && tagId == 20) {
            tagX = 13;
            tagY = 133;
        }
        else if (redMode && tagId == 24) {
            tagX = 131;
            tagY = 133;
        }





        return Math.hypot(tagX - botX, tagY - botY);
    }

    public void switchPipeline(int id) {
        limelight.pipelineSwitch(id);
    }

    public double getBallTy() {
        LLResult result = limelight.getLatestResult();
        if (result==null || !result.isValid()) return 0;

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        double minTy = 1e9;
        for(LLResultTypes.DetectorResult det : detections) {
            if(Math.abs(det.getTargetYDegrees()) < Math.abs(minTy)) {
                minTy = det.getTargetYDegrees();
            }
        }

        /*
        limelight.pipelineSwitch(1); // now we are checking for greens
        for(LLResultTypes.DetectorResult det : detections) {
            if(Math.abs(det.getTargetYDegrees()) < Math.abs(minTy)) {
                minTy = det.getTargetYDegrees();
            }
        }*/

        return minTy == 1e9?0:minTy;
    }
    public boolean tagSeen() {
        if (limelight.getLatestResult() == null || !limelight.getLatestResult().isValid()) {
            return false;
        }

        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        boolean validTagSeen = false;

        tagId = 0;

        for (LLResultTypes.FiducialResult fr : tags) {
            int id = fr.getFiducialId();
            if (id == 20 || id == 24) {
                validTagSeen = true;
                tagId = id;
                break;
            }
        }


        if(tagId == 20 || tagId == 24) {
            return true;
        }
        else {
            return false;
        }
    }
    /**
     * Writes the corrected vision pose (X, Y) into the Pinpoint odometry to correct drift.
     * Call this periodically (e.g. every 500-5000 ms) when a valid AprilTag is visible.
     *
     * Accounts for the Limelight being on a rotating turret: getBotpose() uses a static
     * camera offset that is only accurate when the turret is at zero. This method back-calculates
     * the true robot center using the current turret encoder angle and trig.
     *
     * Set LancersRobot.TURRET_OFFSET_TO_ROBOT_IN to the measured distance (inches) from the
     * turret pivot to the robot's odometry center before using this in competition.
     *
     * @param turretDeg current physical turret angle in degrees (from getCurrentTurretDeg())
     * @return true if a valid tag was seen and the pose was applied, false otherwise.
     */
    public boolean calibratePinpoint(GoBildaPinpointDriver odo, double turretDeg) {
        Pose pose = getBotPose();
        if (pose == null) return false;

        double turretRad = Math.toRadians(turretDeg);
        double cameraHeadingRad = pose.getHeading();

        // Robot heading = camera heading - turret rotation (from encoder, no odo used)
        double driveH = cameraHeadingRad - turretRad;

        // Step 1: camera → turret pivot
        // The camera orbits the turntable center as the turret rotates, so walk back
        // along the camera's current heading to find the pivot position.
        double pivotX = pose.getX() - Math.cos(cameraHeadingRad) * LancersRobot.CAMERA_TO_PIVOT_IN;
        double pivotY = pose.getY() - Math.sin(cameraHeadingRad) * LancersRobot.CAMERA_TO_PIVOT_IN;

        // Step 2: turret pivot → robot odometry center
        // The pivot is offset from the robot center along the robot's forward heading.
        double driveX = pivotX - Math.cos(driveH) * LancersRobot.PIVOT_TO_ROBOT_CENTER_IN;
        double driveY = pivotY - Math.sin(driveH) * LancersRobot.PIVOT_TO_ROBOT_CENTER_IN;

        odo.setPosX(driveX, DistanceUnit.INCH);
        odo.setPosY(driveY, DistanceUnit.INCH);
        return true;
    }

    public void stop() {
        limelight.stop();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
}