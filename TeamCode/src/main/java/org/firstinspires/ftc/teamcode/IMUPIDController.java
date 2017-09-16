package org.firstinspires.ftc.teamcode;

/**
 * Created by adam on 9/16/17.
 */
public class IMUPIDController {


    /**
     * Instance of an IMU interface implementation which
     * will allow us to get imu data.
     */
    IMU imu;


    float P, I, D;
    float pConst, iConst, dConst;

    // Our current "delta time"
    float dT;


    /**
     * Primary constructor
     *
     * @param imu Valid implementation of the IMU interface
     */
    public IMUPIDController(IMU imu) {
        this.imu = imu;
    }

    public double calculatePID() {
        return 0.0;
    }

    public void setTuning(float pTuning, float iTuning, float dTuning) {
        this.pConst = pTuning;
        this.iConst = iTuning;
        this.dConst = dTuning;
    }
}


