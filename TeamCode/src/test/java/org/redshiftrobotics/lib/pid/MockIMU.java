package org.redshiftrobotics.lib.pid;

import org.redshiftrobotics.lib.pid.imu.IMU;

public class MockIMU implements IMU {

    double data;

    public MockIMU() { this(Double.POSITIVE_INFINITY); }
    public MockIMU(double startData) {
        data = startData;
    }

    public double setData(double data) {
        this.data = data;
    }

    @Override
    public double getAngularRotationX() {
        return data;
    }

    @Override
    public double getAngularRotationY(){
        return 0;
    }

    @Override
    public double getAngularRotationZ() {
        return 0;
    }
}
