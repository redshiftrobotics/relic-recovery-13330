package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by adam on 9/16/17.
 */
public class IMUImpl implements IMU {
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;

    public IMUImpl(I2cDeviceSynch imuDevice) {
        imuInit(imuDevice);
    }

    private void imuInit(I2cDeviceSynch device) {
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        BNO055IMU  imu = new AdafruitBNO055IMU(device);
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
