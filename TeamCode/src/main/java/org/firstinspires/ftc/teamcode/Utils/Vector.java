package org.firstinspires.ftc.teamcode.Utils;

public class Vector {

    private double x;
    private double y;


    static public Vector cartesianVector(double x, double y) {
        Vector vec = new Vector();
        vec.setCartesian(x, y);
        return vec;
    }

    static public Vector polarVector(double mag, double ang) {
        Vector vec = new Vector();
        vec.setPolar(mag, ang);
        return vec;
    }

    public void setCartesian(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPolar(double mag, double ang) {
        this.x = mag * Math.sin(ang);
        this.y = mag * Math.cos(ang);
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public double getMag() { return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); }
    public double getAng() {
        final double angle = Math.PI/2 - Math.atan2(y, x);

        if (angle < -Math.PI) {
            return angle + 2 * Math.PI;
        }  else {
            return angle;
        }
    }

    public void shiftAngle(double ang) {
        setPolar(getMag(), getAng() + ang);
    }

    public void add(Vector vec) {
        x += vec.getX();
        y += vec.getY();
    }

    public void scale(double c) {
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