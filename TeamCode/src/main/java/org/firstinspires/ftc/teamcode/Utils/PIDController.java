package org.firstinspires.ftc.teamcode.Utils;

public class PIDController {
    double KP;
    double KD;
    double KI;
    double I_DECAY;

    double integral;
    double lastError;
    long lastTime;

    public PIDController(double KP, double KI, double KD, double I_DECAY) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.I_DECAY = I_DECAY;

        lastError = 0;
        lastTime = -1;
        integral = 0;
    }

    public PIDController(String source) {
        SafeJsonReader myJsonReader = new SafeJsonReader(source);
        this.KP = myJsonReader.getDouble("KP");
        this.KI = myJsonReader.getDouble("KI");
        this.KD = myJsonReader.getDouble("KD");
        this.I_DECAY = myJsonReader.getDouble("I_Decay");

        lastError = 0;
        lastTime = 0;
        integral = 0;
    }

    public double correction(double error) {
        double p = error;

        integral = integral*I_DECAY + error;

        long dt;
        long curtime = System.currentTimeMillis();
        double d;
        if ((dt = (curtime - lastTime)/1000) > 1) {
            d = (lastError - error) / dt;
        } else {
            d = 0;
        }
        lastTime = curtime;
        lastError = error;

        return p*KP + integral*KI + d*KD;
    }

    public void setConstants(double KP, double KI, double KD, double I_DECAY) {
        this.KP = KP;
        this.KD = KD;
        this.KI = KI;
        this.I_DECAY = I_DECAY;
    }

    public double[] getConstants() {
        return new double[] {KP, KI, KD};
    }

    public void resetIntegral() {
        integral = 0;
    }
}
