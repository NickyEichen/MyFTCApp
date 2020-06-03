package org.firstinspires.ftc.teamcode.OpModes;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.HardwareControl.SwerveDrive;
import org.firstinspires.ftc.teamcode.Utils.ButtonManager;
import org.firstinspires.ftc.teamcode.Utils.ButtonStatus;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@TeleOp(name = "PID Tuner")
public class PIDTuner extends LinearOpMode {

    Robot myRobot;
    SwerveDrive mySwerve;

    @Override
    public void runOpMode() {
        myRobot = new Robot(hardwareMap, telemetry);
        mySwerve = myRobot.mySwerve;

        ButtonManager myButtons = new ButtonManager(gamepad1, gamepad2);
        int part = 0;
        float incr = (float) 0.1;
        float[] constants = mySwerve.myModules[0].headingPID.getConstants();

        Vector heading = Vector.cartesianVector(0,0);

        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        myRobot.startAsyncThread();
        while (opModeIsActive()) {
            if (myButtons.x1.isJustOn())
                incr *= 0.1;
            if (myButtons.b1.isJustOn())
                incr *= 10;

            if (myButtons.leftBumper1.isJustOn())
                if (--part < 0) part += 3;
            if (myButtons.rightBumper1.isJustOn())
                if (++part > 2) part -= 3;

            if (myButtons.a1.isJustOn()) {
                constants[part] -= incr;
                for (SwerveDrive.Module m : mySwerve.myModules) {
                    m.headingPID.setConstants(constants[0], constants[1], constants[2], 0.9);
                }
            }

            if (myButtons.y1.isJustOn()) {
                constants[part] += incr;
                for (SwerveDrive.Module m : mySwerve.myModules) {
                    m.headingPID.setConstants(constants[0], constants[1], constants[2], 0.9);
                }
            }

            telemetry.addData("PID Coeffs", ""+constants[0] + ", " + constants[1] + ", " + constants[2]);
            telemetry.addData("Part", part);
            telemetry.addData("Incr", incr);

            telemetry.addData("Encoder Voltage", mySwerve.myModules[3].encoder.getVoltage());
            telemetry.addData("Module Heading", "" + mySwerve.myModules[3].getCurrentAngle() + "  Foward: " + mySwerve.myModules[3].motorIsForward);

            if (gamepad1.dpad_up) heading = Vector.cartesianVector(0, 1);
            if (gamepad1.dpad_down) heading = Vector.cartesianVector(0, -1);
            if (gamepad1.dpad_left) heading = Vector.cartesianVector(-1, 0);
            if (gamepad1.dpad_right) heading = Vector.cartesianVector(1, 0);

            if (gamepad1.left_trigger > 0) heading = Vector.polarVector(-gamepad1.right_stick_y, (float) Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y));

            mySwerve.pointModules(heading);
            telemetry.addData("Target Heading", heading.getAng());

            if (myButtons.rightTrigger.isJustOn())
                mySwerve.myModules[0].headingPID.saveConstants();

            if (gamepad2.y)
                mySwerve.zeroModulePositions();

            myButtons.update();
            telemetry.update();
            myRobot.update();
        }
        myRobot.killAsyncThread();

    }
}
