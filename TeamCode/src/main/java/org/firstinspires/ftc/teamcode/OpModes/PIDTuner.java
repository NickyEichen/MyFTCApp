package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.SwerveDrive;

@TeleOp(name = "PID Tuner")
public class PIDTuner extends LinearOpMode {

    Gyro myGyro;
    SwerveDrive mySwerve;

    @Override
    public void runOpMode() {
        myGyro = new Gyro(hardwareMap);
        mySwerve = new SwerveDrive(hardwareMap, telemetry, myGyro);

        waitForStart();

        while (opModeIsActive()) {

        }

    }
}
