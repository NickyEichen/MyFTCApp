package org.firstinspires.ftc.teamcode.HardwareControl;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.PIDController;
import org.firstinspires.ftc.teamcode.Utils.SafeJsonReader;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class SwerveDrive {

    SafeJsonReader zeroPositions;

    public Module[] myModules = new Module[4];
    String[] modulePositions = {"fr", "br", "bl", "fl"};
    Gyro myGyro;

    boolean fieldCentric = true;


    static final double ROTATION_SCALING = 1;
    final static double MAX_WHEEL_VELOCITY = 2.25; // m/s
    final static double MODULE_RADIUS = 0.2604274275110054; //m

    public SwerveDrive(HardwareMap hwMap, Telemetry telemetry, Gyro myGyro) {
        for (int i = 0; i < 4; i++)
            myModules[i] = new Module(hwMap, telemetry, modulePositions[i]);

        this.myGyro = myGyro;
    }

    public boolean pointModules(Vector heading) {
        double angle = heading.getAng() - (fieldCentric? getOrientation(): 0);

        boolean isFinished = true;
        for (Module module : myModules)
            isFinished = isFinished && module.steerModule(heading.getAng());
        return isFinished;
    }

    public void drivePower(Vector heading, double rotation) {
        Vector[] rotVectors = { Vector.polarVector(rotation, 3/4*Math.PI),
                                Vector.polarVector(rotation, 7/4*Math.PI),
                                Vector.polarVector(rotation, 9/4*Math.PI),
                                Vector.polarVector(rotation, 1/4*Math.PI)};

        if (fieldCentric) {
            double shift = -getOrientation();
            for (Vector v : rotVectors)
                v.shiftAngle(shift);
            heading.shiftAngle(shift);
        }

        Vector[] moduleVectors = new Vector[4];
        for (int i = 0; i < 4; i++) {
            moduleVectors[i] = heading.copy();
            moduleVectors[i].add(rotVectors[i]);
        }

        normalizeVectors(moduleVectors, 1);

        for (int i =0; i < 4; i++)
            myModules[i].drivePower(moduleVectors[i]);
    }

    public void driveVelocity(Vector heading, double rotation) {
        double moduleSpeed = rotation * MODULE_RADIUS;
        Vector[] rotVectors = { Vector.polarVector(moduleSpeed, 3/4*Math.PI),
                                Vector.polarVector(moduleSpeed, 7/4*Math.PI),
                                Vector.polarVector(moduleSpeed, 9/4*Math.PI),
                                Vector.polarVector(moduleSpeed, 1/4*Math.PI)};

        if (fieldCentric) {
            double shift = -getOrientation();
            for (Vector v : rotVectors)
                v.shiftAngle(shift);
            heading.shiftAngle(shift);
        }

        Vector[] moduleVectors = new Vector[4];
        for (int i = 0; i < 4; i++) {
            moduleVectors[i] = heading.copy();
            moduleVectors[i].add(rotVectors[i]);
        }

        if (normalizeVectors(moduleVectors, MAX_WHEEL_VELOCITY))
            Log.e("SWERVE_DRIVE", "Wheel velocity exceeded maximum. Scaled down");

        for (int i =0; i < 4; i++)
            myModules[i].driveVelocity(moduleVectors[i]);
    }

    private double getOrientation() {
        return myGyro.getHeading();
    }

    /** Destructively scales VECTORS to double MAX. */
    private boolean normalizeVectors(Vector[] vectors, double max) {
        double maxMag = 0;
        for (Vector v : vectors) {
            maxMag = Math.max(maxMag, v.getMag());
        }
        if (maxMag > max) {
            double scale = max/maxMag;
            for (Vector v:vectors)
                v.scale(scale);
            return true;
        }
        return false;
    }

    /** Class represents one swerve module. */
    public class Module {
        DcMotorEx motor;
        CRServo servo;
        AnalogInput encoder;
        public PIDController headingPID;

        boolean motorIsForward = true;
        double zeroPosition;

        static final double ENCODER_MAX_VOLTAGE = 3.23;
        static final double MAX_HEADING_ERROR = 0.05;

        //For driveVelocity
        static final double RADIUS = 0.0036; // (meters)
        static final double GEAR_RATIO = 10.736842105263158; // Reduction


        Module(HardwareMap hwMap, Telemetry telemetry, String name) {
            motor = hwMap.get(DcMotorEx.class, name + "Motor");
            servo = hwMap.crservo.get(name + "Servo");
            encoder = hwMap.analogInput.get(name + "Encoder");

            headingPID = new PIDController("SwerveSteeringPID");
            zeroPosition = zeroPositions.getDouble(name);
        }

        /** Steers module towards ANGLE.
         ** Returns TRUE if module has reached position, false otherwise. */
        boolean steerModule(double angle) {
            double error = negNormPi(getHeading() - angle);

            if (Math.abs(error) > Math.PI/2) {
                switchDirection();
                error = negNormPi(getHeading() - angle);
            }

            double servoPow = negNormOne(headingPID.correction(error));
            servo.setPower(servoPow);

            if (error > MAX_HEADING_ERROR)
                return false;
            else
                return true;
        }

        /** Drives module along Vector VEC. Interprets vector as power. */
        void drivePower(Vector vec) {
            steerModule(vec.getAng());
            motor.setPower(vec.getMag() * (motorIsForward ? 1 : -1));
        }

        //* Drives module along Vector VEC. Interprets vector as velocity in m/s. */
        void driveVelocity(Vector vec) {
            steerModule(vec.getAng());
            double vel = vec.getMag() * (motorIsForward? 1: -1);
            double angularRate = vel * RADIUS * GEAR_RATIO;
            motor.setVelocity(angularRate, AngleUnit.RADIANS);
        }


        /** Returns the current heading of the module. */
        double getHeading() {
            double heading = (1 - encoder.getVoltage()/ENCODER_MAX_VOLTAGE) * 2*Math.PI;
            if (!motorIsForward)
                heading += Math.PI;

            if (heading > Math.PI) {
                return heading - 2*Math.PI;
            } else {
                return heading;
            }
        }

        private void switchDirection() {
            motorIsForward = !motorIsForward;
            headingPID.resetIntegral();
        }

        private void stopModule() {
            motor.setMotorDisable();
        }
    }

    private double negNormPi(double val) {
        return negNorm(val, Math.PI);
    }

    private double negNormOne(double val) {
        return negNorm(val, 1);
    }

    private double negNorm(double val, double norm) {
        while (val > norm) {
            val -= norm*2;
        }
        while (val < - norm) {
            val += norm*2;
        }
        return val;
    }
}
