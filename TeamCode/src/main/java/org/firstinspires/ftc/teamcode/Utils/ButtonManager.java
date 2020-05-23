package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ButtonManager {

    Gamepad gp1, gp2;

    public ButtonStatus a1 = new ButtonStatus();
    public ButtonStatus b1 = new ButtonStatus();
    public ButtonStatus x1 = new ButtonStatus();
    public ButtonStatus y1 = new ButtonStatus();

    public ButtonStatus leftBumper1 = new ButtonStatus();
    public ButtonStatus rightBumper1 = new ButtonStatus();


    public ButtonManager(Gamepad gamepad1, Gamepad gamepad2) {
        gp1 = gamepad1;
        gp2 = gamepad2;
    }

    public void update() {
        a1.recordNewValue(gp1.a);
        b1.recordNewValue(gp1.b);
        x1.recordNewValue(gp1.x);
        y1.recordNewValue(gp1.y);

        leftBumper1.recordNewValue(gp1.left_bumper);
        rightBumper1.recordNewValue(gp1.right_bumper);
    }
}
