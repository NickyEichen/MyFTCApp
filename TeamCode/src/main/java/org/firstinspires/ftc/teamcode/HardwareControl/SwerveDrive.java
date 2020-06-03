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
import org.firstinspires.ftc.teamcode.Utils.UpdateLoopController;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class SwerveDrive {

    SafeJsonReader zeroPositions;

    public Module[] myModules = new Module[4];
    private static final String[] modulePositions = {"fr", "br", "bl", "fl"};
    private final Gyro myGyro;



    private static final float ROTATION_SCALING = 1;
    private final static float MAX_WHEEL_VELOCITY = (float) 2.25; // m/s
    private final static float MODULE_RADIUS = (float) 0.2604274275110054; //m
    private final static float PI = (float) Math.PI;

    private final static String LOG = "SwerveController_9773";
    private final static boolean DEBUG = true;


    private boolean fieldCentric = true;
    private Vector targetHeading = Vector.cartesianVector(0, 0);
    private float targetRotation = 0;
    private boolean targetIsPower = true;

    // INIT
    public SwerveDrive(HardwareMap hwMap, Telemetry telemetry, Gyro myGyro) {
        zeroPositions = new SafeJsonReader("SwerveZeroPositions");
        this.myGyro = myGyro;

        for (int i = 0; i < 4; i++)
            myModules[i] = new Module(hwMap, telemetry, modulePositions[i]);
    }

    // Public Functions
    public boolean pointModules(Vector heading) {
        targetHeading.setCartesian(0,0);
        targetRotation = 0;


        float angle = heading.getAng() - (fieldCentric? getRobotOrientation(): 0);

        boolean isFinished = true;
        for (Module module : myModules) {
            module.setHeading(heading.getAng());
            isFinished = isFinished && module.reachedTarget();
        }
        return isFinished;
    }

    public void update() {
        if (targetIsPower)
            drivePower(targetHeading, targetRotation);
        else
            driveVelocity(targetHeading, targetRotation);
    }

    public void setTargetPower(Vector heading, float rotation) {
        targetHeading = heading;
        targetRotation = rotation;
        targetIsPower = true;
    }

    public void setTargetVelocity(Vector heading, float rotation) {
        targetHeading = heading;
        targetRotation = rotation;
        targetIsPower = false;
    }

    // Internal Drive Functions
    private void drivePower(Vector heading, float rotation) {
        if (heading.getMag() == 0 && rotation == 0) return;

        Vector[] rotVectors = { Vector.polarVector(rotation, (float) 3/4*PI),
                                Vector.polarVector(rotation, (float) 7/4*PI),
                                Vector.polarVector(rotation, (float) 9/4*PI),
                                Vector.polarVector(rotation, (float) 1/4*PI)};

        if (fieldCentric) {
            float shift = -getRobotOrientation();
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

    private void driveVelocity(Vector heading, float rotation) {
        if (heading.getMag() == 0 && rotation == 0) return;

        float moduleSpeed = rotation * MODULE_RADIUS;
        Vector[] rotVectors = { Vector.polarVector(moduleSpeed, (float)3/4*PI),
                                Vector.polarVector(moduleSpeed, (float)7/4*PI),
                                Vector.polarVector(moduleSpeed, (float)9/4*PI),
                                Vector.polarVector(moduleSpeed, (float)1/4*PI)};

        if (fieldCentric) {
            float shift = -getRobotOrientation();
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

    private float getRobotOrientation() {
        return myGyro.getHeading();
    }

    /** Destructively scales VECTORS to float MAX. */
    private boolean normalizeVectors(Vector[] vectors, float max) {
        float maxMag = 0;
        for (Vector v : vectors) {
            maxMag = Math.max(maxMag, v.getMag());
        }
        if (maxMag > max) {
            float scale = max/maxMag;
            for (Vector v:vectors)
                v.scale(scale);
            return true;
        }
        return false;
    }

    /** Class represents one swerve module. */
    public class Module extends UpdateLoopController {
        DcMotorEx motor;
        CRServo servo;
        public AnalogInput encoder;
        public PIDController headingPID;
        String name;

        public boolean motorIsForward = true;
        float zeroPosition = 0;

        static final float ENCODER_MAX_VOLTAGE = (float) 5.0;
        static final float MAX_HEADING_ERROR = (float) 0.05;

        //For driveVelocity
        static final float RADIUS = (float) 0.0036; // (meters)
        static final float GEAR_RATIO = (float) 10.736842105263158; // Reduction

        // For object permenance :) jkjk saves current state
        private float currentAngle = 0;
        private float targetAngle = 0;


        Module(HardwareMap hwMap, Telemetry telemetry, String name) {
            motor = hwMap.get(DcMotorEx.class, name + "Motor");
            servo = hwMap.crservo.get(name + "Servo");
            encoder = hwMap.analogInput.get(name + "Encoder");

            headingPID = new PIDController("SwerveSteeringPID");
            zeroPosition = (float) zeroPositions.getDouble(name);
            this.name = name;
        }

        /** Overrides update from UpdateLoopController.
         * Updates loop to point towards targetangle.
         * Saves whether it has reached its heading. */
        @Override
        public void update() {
            currentAngle = getHeading();
            float error = negNormPi(currentAngle - targetAngle);

            if (Math.abs(error) > PI/2) {
                switchDirection();
                error = negNormPi(getHeading() - targetAngle);
            }

            float servoPow = minMaxOne(headingPID.correction(error, DEBUG && name.equals("fl")));
            servo.setPower(servoPow);
            //if (DEBUG && name.equals("fl")) Log.i(LOG, "Motor Fowards: " + motorIsForward + "  Error: " + error + "  Power: " + servoPow + "  DT: " + headingPID.dt);
        }

        public boolean reachedTarget() {
            return Math.abs(negNormPi(targetAngle - currentAngle)) < MAX_HEADING_ERROR;
        }

        public float getCurrentAngle() {
            return currentAngle;
        }

        private void setHeading(float angle) {
            targetAngle = angle;
        }

        /** Drives module along Vector VEC. Interprets vector as power. */
        void drivePower(Vector vec) {
            setHeading(vec.getAng());
            motor.setPower(vec.getMag() * (motorIsForward ? 1 : -1));
        }

        //* Drives module along Vector VEC. Interprets vector as velocity in m/s. */
        void driveVelocity(Vector vec) {
            setHeading(vec.getAng());
            float vel = vec.getMag() * (motorIsForward? 1: -1);
            float angularRate = vel * RADIUS * GEAR_RATIO;
            motor.setVelocity(angularRate, AngleUnit.RADIANS);
        }


        /** Returns the current heading of the module. */
        private float getHeading() {
            float heading = (1 - ((float) encoder.getVoltage())/ENCODER_MAX_VOLTAGE) * 2*PI;
            if (!motorIsForward)
                heading += PI;

            if (heading > PI) {
                return heading - 2*PI;
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

        private void setZeroPosition() {
            zeroPosition = getHeading();
            zeroPositions.modifyDouble(name, zeroPosition);
        }
    }

    public void zeroModulePositions() {
        for (Module m : myModules)
            m.setZeroPosition();
        zeroPositions.updateFile();
    }


    private float negNormPi(float val) {
        return negNorm(val, PI);
    }

    private float minMaxOne(float val) {
        if (val > 1) return 1;
        if (val < -1) return -1;
        return val;
    }

    private float negNorm(float val, float norm) {
        while (val > norm) {
            val -= norm*2;
        }
        while (val < - norm) {
            val += norm*2;
        }
        return val;
    }
}
