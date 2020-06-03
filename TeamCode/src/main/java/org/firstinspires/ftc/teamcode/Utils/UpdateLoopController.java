package org.firstinspires.ftc.teamcode.Utils;

import android.util.Log;

import java.util.ArrayList;
import java.util.List;

public abstract class UpdateLoopController {

    private static List<UpdateLoopController> all = new ArrayList<>();


    public UpdateLoopController() {
        all.add(this);
    }

    public abstract void update();
    public abstract String getName();

    public static void updateAll() {
        Log.i("UpdateLoop", "Time is: " + System.currentTimeMillis());
        for (UpdateLoopController obj : all)
            obj.update();
    }
}
