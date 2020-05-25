package org.firstinspires.ftc.teamcode.Utils;

public class Vector {

    private float x;
    private float y;

    private static final float PI = (float) Math.PI;


    static public Vector cartesianVector(float x, float y) {
        Vector vec = new Vector();
        vec.setCartesian(x, y);
        return vec;
    }

    static public Vector polarVector(float mag, float ang) {
        Vector vec = new Vector();
        vec.setPolar(mag, ang);
        return vec;
    }

    public void setCartesian(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public void setPolar(float mag, float ang) {
        this.x = mag * (float) Math.sin(ang);
        this.y = mag * (float) Math.cos(ang);
    }

    public float getX() { return x; }
    public float getY() { return y; }

    public float getMag() { return (float) Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); }
    public float getAng() {
        final float angle = PI/2 - (float) Math.atan2(y, x);

        if (angle < -PI) {
            return angle + 2 * PI;
        }  else {
            return angle;
        }
    }

    public void shiftAngle(float ang) {
        setPolar(getMag(), getAng() + ang);
    }

    public void add(Vector vec) {
        x += vec.getX();
        y += vec.getY();
    }

    public void scale(float c) {
        setPolar(getMag()*c, getAng());
    }

    public Vector copy() {
        return cartesianVector(x, y);
    }

    public Vector addCopy(Vector v) {
        Vector newVector = copy();
        newVector.add(v);
        return newVector;
    }
}