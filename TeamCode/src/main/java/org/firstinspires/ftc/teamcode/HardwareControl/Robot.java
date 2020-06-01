package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.UpdateLoopThread;

public class Robot {

    private Telemetry telemetry;
    private Gyro myGyro;
    private SwerveDrive mySwerve;
    private UpdateLoopThread secondThread = new UpdateLoopThread();

    public Robot(HardwareMap hwMap, Telemetry telemetry) {
        myGyro = new Gyro(hwMap);
        mySwerve = new SwerveDrive(hwMap, telemetry, myGyro);
        this.telemetry = telemetry;
    }

    public void update() {
        myGyro.update();
        mySwerve.update();
    }

    public void startAsyncThread() {
        secondThread.start();
    }
    public void killAsyncThread() {
        secondThread.exit();
    }
}
