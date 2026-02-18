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
    public List<String> acceptedBallLabels = Arrays.asList(new String[] {"blue", "red", "yellow"});

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
        limelight.pipelineSwitch(8); // TODO: Change this pipeline to the correct pipeline later
        limelight.start();
    }

    // returns a 2D pose with heading (in radians) if an april tag is found
    public Pose getBotPose() {

        if (limelight.getLatestResult() == null || !limelight.getLatestResult().isValid()) {
            return null;
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

    double tagX;
    double tagY;

    public double getDistanceToTag() {
        Pose pose = this.getBotPose();

        if (pose == null) {
            return 0.0;
        }

        double botX = pose.getX();
        double botY = pose.getY();

        //replace these values with the coordinates of the actual apriltags respectively
        if (tagId == 20) // blue
        {
            //replace these numbers MAKE SURE TO USE INCHES
            tagX = 13;
            tagY = 133;
        }
        else if (tagId == 24) { // red
            //replace these numbers MAKE SURE TO USE INCHES
            tagX = 131;
            tagY = 133;
        }



        return Math.hypot(tagX - botX, tagY - botY);
    }

    public double getBallTy() {
        LLResult result = limelight.getLatestResult();
        if (result==null || !result.isValid()) return 0;

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        double minTy = 1e9;
        for(LLResultTypes.DetectorResult det : detections) {
            if(acceptedBallLabels.contains(det.getClassName())) {
                if(Math.abs(det.getTargetXDegrees()) < Math.abs(minTy)) {
                    minTy = det.getTargetXDegrees();
                }
            }
        }
        return minTy == 1e9 ? 0 : minTy;
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