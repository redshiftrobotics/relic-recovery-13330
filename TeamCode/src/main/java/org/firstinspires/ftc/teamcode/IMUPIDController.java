package org.firstinspires.ftc.teamcode;

import java.util.*;

/**
 * Created by adam on 9/16/17.
 */

//TODO: Unit test! 
public class IMUPIDController {
    static boolean newIMU = false;
    /**
     * Instance of an IMU interface implementation which
     * will allow us to get imu data.
     */
    IMU imu;


    // Proportional, Integral, and Derivative Terms of the PID formula
    float P = 0f, I = 0f, D = 0f;

    // Proportional, Integral, and Derivative Constants for Tuning
    float pConst = 1f, iConst = 1f, dConst = 1f;

    float lastError;

    float target;

    // Our current delta time that holds the time between current and last calculations.
    float dT;


    /**
     * Primary constructor
     *
     * @param imu Valid implementation of the IMU interface
     */
    public IMUPIDController(IMU imu) {
        this.imu = imu;
    }

    /**
     * Calculates the current imu
     */
    public void calculateP() {
        float angle = newIMU ? imu.getAngularRotationX() + 180: imu.getAngularRotationX();
        lastError = P;

        System.out.println("Angle is " + angle);
        System.out.println("Target is " + target);

        //angle = 359
        //target = 1

        if (angle + 360 - target <= 180) {
            P = (angle -  target + 360);
        } else if (target + 360 - angle <= 180) {
            P = (target - angle + 360) * -1;
        } else if (angle -  target <= 180) {
            P = (angle -  target);
        }
        //System.out.println("P is" + P);
    }

    public void calculateI() {
        I += P * dT / 1000;
    }

    // d e(t) / dt
    public void calculateD() {
        D = (P - lastError) / (dT/1000);
    }

    public double calculatePID(long deltaTime) {
        dT = deltaTime;
        calculateP();
        calculateI();
        calculateD();

        return pConst * P + iConst * I / 2000 + dConst * D / 2000;
    }

    public void setTuning(float pTuning, float iTuning, float dTuning) {
        this.pConst = pTuning;
        this.iConst = iTuning;
        this.dConst = dTuning;
    }

    public void setTarget(float targetAngle) {
        this.target = targetAngle;
    }

    public void addTarget(float angleDelta) {
        this.target += angleDelta;
    }
    public void resetTarget() {
        this.target = this.imu.getAngularRotationX();
    }

    public void clearData() {
        this.P = 0;
        this.I = 0;
        this.D = 0;
        this.lastError = 0;
        this.dT = 0;
        //this.target = 0;
    }
}

