package org.firstinspires.ftc.teamcode;

import java.util.*;

/**
 * Created by adam on 9/16/17.
 */
public class IMUPIDController {
    /**
     * Instance of an IMU interface implementation which
     * will allow us to get imu data.
     */
    IMU imu;


    // Proportional, Integral, and Derivative Terms of the PID formula
    float P = 0f, I = 0f, D = 0f;

    // Proportional, Integral, and Derivative Constants for Tuning
    float pConst, iConst, dConst;

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
        float angle = imu.getAngularRotationX();
        lastError = P;

        if (angle + 360 - target <= 180) {
            P = (angle -  target + 360);
        } else if (target + 360 - angle <= 180) {
            P = (target - angle + 360) * -1;
        } else if (angle -  target <= 180) {
            P = (angle -  target);
        }
    }

    public void calculateI() {
        I += P * dT;
    }

    // d e(t) / dt
    public void calculateD() {
        D = (P - lastError) / dT;
    }

    public double calculatePID(long deltaTime) {
        dT = deltaTime;
        calculateP();
        calculateI();
        calculateD();

        return pConst * P + iConst * I + dConst * D;
    }

    public void setTuning(float pTuning, float iTuning, float dTuning) {
        this.pConst = pTuning;
        this.iConst = iTuning;
        this.dConst = dTuning;
    }
}


