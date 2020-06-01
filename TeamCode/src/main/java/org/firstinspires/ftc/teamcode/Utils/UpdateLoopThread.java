package org.firstinspires.ftc.teamcode.Utils;

public class UpdateLoopThread extends Thread {

    boolean stop = false;

    public void run() {
        while (!stop) {
            UpdateLoopController.updateAll();
        }
    }

    public void exit() {
        stop = true;
    }
}
