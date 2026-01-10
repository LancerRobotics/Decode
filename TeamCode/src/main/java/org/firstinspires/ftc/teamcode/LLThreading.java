package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
// DO NOT USE
public class LLThreading implements Runnable {
    LancersRobot robot;

    public void run() {
        robot.aimOuttakeToTx(0.5);
    }


    public LLThreading(LancersRobot robot) {
        this.robot = robot;

        LLThreading runnable = new LLThreading(this.robot);
        Thread thread = new Thread(runnable);
        thread.start();
    }

}
