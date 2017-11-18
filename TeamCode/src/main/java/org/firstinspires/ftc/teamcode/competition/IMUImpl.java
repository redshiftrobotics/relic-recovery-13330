package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Created by adam on 9/16/17.
 */
public class IMUImpl implements IMU {
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;

    public IMUImpl(BNO055IMU imu) {
        imuInit(imu);
    }

    private void imuInit(BNO055IMU imu) {
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

      //  BNO055IMU  imu = new AdafruitBNO055IMU(device);
        imu.initialize(imuParameters);
        this.imu = imu;
    }

    @Override
    public float getAngularRotationX() {
        return this.imu.getAngularOrientation().firstAngle;
    }

    @Override
    public float getAngularRotationY() {
        return this.imu.getAngularOrientation().secondAngle;
    }

    @Override
    public float getAngularRotationZ() {
        return this.imu.getAngularOrientation().thirdAngle;
    }
}
