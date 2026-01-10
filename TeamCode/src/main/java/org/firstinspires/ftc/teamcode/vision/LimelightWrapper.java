package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class LimelightWrapper {

    private final Limelight3A limelight;

    // center of an FTC field (in inches) for pedro is (72, 72), for limelight its (0,0)
    private static final double PEDRO_X_OFFSET_IN = 72.0;
    private static final double PEDRO_Y_OFFSET_IN = 72.0;
    private static int tagId;

    public LimelightWrapper(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LancersBotConfig.LIMELIGHT);
        limelight.pipelineSwitch(8); // TODO: Change this pipeline to the correct pipeline later
        limelight.start();
    }

    // returns a 2D pose with heading (in radians) if an april tag is found
    public Pose getBotPose() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        boolean validTagSeen = false;

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
        else if (tagId == 24) { // reg
            //replace these numbers MAKE SURE TO USE INCHES
            tagX = 131;
            tagY = 133;
        }



        return Math.hypot(tagX - botX, tagY - botY);
    }
    public boolean tagSeen() {
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