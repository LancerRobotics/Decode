package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

//This code was written using a tutorial by Brogan M. Pratt
//Access the tutorial at: https://https://www.youtube.com/watch?v=OZt33z-lyYo
//Many classes from ConceptAprilTag.java by FTC are used


//Wow so much cool stuff
public class AprilTagScanner {

    private org.firstinspires.ftc.vision.apriltag.AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> listOfDetectedTags = new ArrayList<>();
    private Telemetry telemetry;

    //Initializes the telemetry to display detected April Tags on screen
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    //Adds detected April Tags to the list
    public void update() {
        listOfDetectedTags = aprilTagProcessor.getDetections();
    }

    //Initializes a list of detected tags
    public List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> getListOfDetectedTags() {
        return listOfDetectedTags;
    }

    //Displays the information gathered from the April Tag to the screen
    public void displayTelemetry(AprilTagDetection detectedTagId) {
        if (detectedTagId == null) {
            return;
        }
        if (detectedTagId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedTagId.id, detectedTagId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedTagId.ftcPose.x, detectedTagId.ftcPose.y, detectedTagId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedTagId.ftcPose.pitch, detectedTagId.ftcPose.roll, detectedTagId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedTagId.ftcPose.range, detectedTagId.ftcPose.bearing, detectedTagId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedTagId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedTagId.center.x, detectedTagId.center.y));
        }
    }

    //Loops through the detected tags to find which tags they are
    public org.firstinspires.ftc.vision.apriltag.AprilTagDetection getTagById(int id) {
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : listOfDetectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    //Stops the vision portal just in case
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}