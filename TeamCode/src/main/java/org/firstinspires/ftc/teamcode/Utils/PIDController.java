package org.firstinspires.ftc.teamcode.Utils;

import android.util.Log;

public class PIDController {
    private float KP;
    private float KD;
    private float KI;
    private float I_DECAY;

    private float integral;
    private float lastError;
    private long lastTime;
    public float dt = 0;

    private String source;

    public PIDController(float KP, float KI, float KD, float I_DECAY) {
        setConstants(KP, KI, KD, I_DECAY);

        lastError = 0;
        lastTime = -1;
        integral = 0;
    }

    public PIDController(double KP, double KI, double KD, double I_DECAY) {
        this((float) KP, (float) KI, (float) KD, (float) I_DECAY);
    }

    public PIDController(String source) {
        SafeJsonReader myJsonReader = new SafeJsonReader(source);
        this.KP = (float) myJsonReader.getDouble("KP");
        this.KI = (float) myJsonReader.getDouble("KI");
        this.KD = (float) myJsonReader.getDouble("KD");
        this.I_DECAY = (float) myJsonReader.getDouble("I_DECAY");

        this.source = source;

        lastError = 0;
        lastTime = 0;
        integral = 0;
    }

    public float correction(float error, boolean debug) {
        float p = error;

        integral = integral*I_DECAY + error;

        float dt;
        long curtime = System.currentTimeMillis();
        float d;
        if ((dt = (float) (curtime - lastTime)/1000) < 5) {
            d = (lastError - error) / dt;
        } else {
            d = 0;
        }
        lastTime = curtime;
        lastError = error;

        this.dt = dt;

        if (debug) Log.i("Swerve_PID", "Error: " + error + "  Power: " + (p*KP + integral*KI + d*KD) + "  P: " + p*KP + "  I: " + integral*KI + "  D: " + d*KD);

        return p*KP + integral*KI + d*KD;
    }

    public void setConstants(float KP, float KI, float KD, float I_DECAY) {
        this.KP = KP;
        this.KD = KD;
        this.KI = KI;
        this.I_DECAY = I_DECAY;
    }

    public void setConstants(double KP, double KI, double KD, double I_DECAY) {
        setConstants((float) KP, (float) KI, (float) KD, (float) I_DECAY);
    }

    public float[] getConstants() {
        return new float[] {KP, KI, KD};
    }

    public void resetIntegral() {
        integral = 0;
    }

    public void saveConstants() {
        if (source == null)
            return;
        SafeJsonReader myJsonReader = new SafeJsonReader(source);
        myJsonReader.modifyDouble("KP", KP);
        myJsonReader.modifyDouble("KI", KI);
        myJsonReader.modifyDouble("KD", KD);
        myJsonReader.modifyDouble("I_DECAY", I_DECAY);
    }
}
