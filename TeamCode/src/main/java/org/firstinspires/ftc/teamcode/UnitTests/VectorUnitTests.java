package org.firstinspires.ftc.teamcode.UnitTests;


import org.firstinspires.ftc.teamcode.Utils.Vector;

import org.junit.Assert;
import org.junit.Test;


public class VectorUnitTests {

    static Vector v1 = Vector.cartesianVector(1, 1);
    static Vector v2 = Vector.cartesianVector(1, 0);
    static Vector v3 = Vector.cartesianVector(0, 1);

    static final float PI = (float) Math.PI;

    @Test
    public void testVectorAddition() {
        Assert.assertTrue(vectorEquals(v1, v2.addCopy(v3)));
    }

    @Test
    public void testPolarVectors() {
        Vector v2Copy = v2.copy();
        v2Copy.shiftAngle(-PI/2);
        Assert.assertTrue(vectorEquals(v3, v2Copy));
    }

    @Test
    public void testPolarVectors2() {
        Vector v1 = Vector.cartesianVector(0, 1);
        Vector v2 = Vector.polarVector(1, 0);
        Assert.assertTrue(vectorEquals(v1, v2));
    }

    private boolean vectorEquals(Vector v1, Vector v2) {
        if (Math.abs(v2.getX() - v1.getX()) > 0.01)
            return false;
        if (Math.abs(v2.getY() - v1.getY()) > 0.01)
            return false;
        return true;
    }
}
