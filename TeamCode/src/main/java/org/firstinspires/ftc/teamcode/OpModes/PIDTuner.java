package org.firstinspires.ftc.teamcode.OpModes;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.SwerveDrive;
import org.firstinspires.ftc.teamcode.Utils.ButtonManager;
import org.firstinspires.ftc.teamcode.Utils.ButtonStatus;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@TeleOp(name = "PID Tuner")
public class PIDTuner extends LinearOpMode {

    Gyro myGyro;
    SwerveDrive mySwerve;

    @Override
    public void runOpMode() {
        myGyro = new Gyro(hardwareMap);
        mySwerve = new SwerveDrive(hardwareMap, telemetry, myGyro);

        ButtonManager myButtons = new ButtonManager(gamepad1, gamepad2);
        int part = 0;
        float incr = (float) 0.1;
        float[] constants = mySwerve.myModules[0].headingPID.getConstants();

        Vector heading = Vector.cartesianVector(0,0);

        waitForStart();
        while (opModeIsActive()) {
            if (myButtons.a1.isJustOn())
                incr *= 0.1;
            if (myButtons.y1.isJustOn())
                incr *= 10;

            if (myButtons.leftBumper1.isJustOn())
                if (--part < 0) part += 3;
            if (myButtons.rightBumper1.isJustOn())
                if (++part > 2) part -= 3;

            if (myButtons.x1.isJustOn()) {
                constants[part] -= incr;
                for (SwerveDrive.Module m : mySwerve.myModules) {
                    m.headingPID.setConstants(constants[0], constants[1], constants[2], 0.9);
                }
            }

            if (myButtons.b1.isJustOn()) {
                constants[part] += incr;
                for (SwerveDrive.Module m : mySwerve.myModules) {
                    m.headingPID.setConstants(constants[0], constants[1], constants[2], 0.9);
                }
            }

            telemetry.addData("PID Coeffs", ""+constants[0] + ", " + constants[1] + ", " + constants[2]);
            telemetry.addData("Part", part);
            telemetry.addData("Incr", incr);

            float h = (float) Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Target heading", h);

            if (gamepad1.left_trigger > 0) {
                heading = Vector.polarVector(1, h);
            }

            mySwerve.pointModules(heading);

            myButtons.update();
            telemetry.update();
        }

    }
}
