package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

import com.pedropathing.geometry.Pose;

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

    // returns a 2D pose with heading (in radians) if an april tag is found
    public Pose getBotPose() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

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

        if (!validTagSeen) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        // convert meters to inches
        double xIn = botpose.getPosition().x * 39.37;
        double yIn = botpose.getPosition().y * 39.37;

        // Convert yaw degrees -> radians for pedro
        double headingRad = Math.toRadians(botpose.getOrientation().getYaw());

        xIn += PEDRO_X_OFFSET_IN;
        yIn += PEDRO_Y_OFFSET_IN;

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
    public void stop() {
        limelight.stop();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
}