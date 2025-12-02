package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//This whole thing basically just implements the code in AprilTagScanner.java

/*
This code identifies an April Tag on the webcam stream
When it is run, it provides the following info:
1. The XYZ: the distance on each axis in inches from the April Tag
2. The PRY: pitch, roll, and yaw in relation to the April Tag in degrees
3. The RBE: range (distance from center of camera to center of tag in inches),
   bearing (angle of deflection from the tag in degrees), and
   elevation (how far up you are compared to the tag in degrees)

Those readings are not 100% perfect
 */

//Wow more cool stuff
@Autonomous
public class AprilTagScannerForAuton extends OpMode {

    AprilTagScanner aprilTagScanner = new AprilTagScanner();

    @Override
    public void init() {
        aprilTagScanner.init(hardwareMap, telemetry);
    }

    public void loop() {
        aprilTagScanner.update();

        //Looks for tag ID 20, which indicates a blue alliance goal
        //THIS NEEDS TO BE UPDATED DEPENDING ON WHICH APRIL TAG WE ARE LOOKING FOR
        AprilTagDetection id20 = aprilTagScanner.getTagById(20);
        aprilTagScanner.displayTelemetry(id20);
    }

}