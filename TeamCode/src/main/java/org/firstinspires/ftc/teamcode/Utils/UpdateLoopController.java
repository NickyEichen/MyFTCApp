package org.firstinspires.ftc.teamcode.Utils;

import java.util.List;

public abstract class UpdateLoopController {

    private static List<UpdateLoopController> all;

    public UpdateLoopController() {
        all.add(this);
    }

    public abstract void update();

    public static void updateAll() {
        for (UpdateLoopController obj : all)
            obj.update();
    }
}
