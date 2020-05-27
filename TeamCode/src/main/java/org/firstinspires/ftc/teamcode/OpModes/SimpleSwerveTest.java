package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.ButtonManager;

@TeleOp(name="Servo Debug")
public class SimpleSwerveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        AnalogInput encoder = hardwareMap.analogInput.get("flEncoder");
        CRServo servo = hardwareMap.crservo.get("flServo");
        ButtonManager myButtons = new ButtonManager(gamepad1, gamepad2);

        waitForStart();

        float power = 0;


        while (opModeIsActive()) {
            telemetry.addData("Voltage", encoder.getVoltage());

            telemetry.addData("Servo Power", power);
            servo.setPower(power);

            if (myButtons.a1.isJustOn()) power -= 0.1;
            if (myButtons.y1.isJustOn()) power += 0.1;
            if (myButtons.x1.isJustOn()) power -= 0.01;
            if (myButtons.b1.isJustOn()) power += 0.01;


            myButtons.update();
            telemetry.update();
        }
    }
}
