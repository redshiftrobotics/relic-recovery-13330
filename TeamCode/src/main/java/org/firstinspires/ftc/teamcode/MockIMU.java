package org.firstinspires.ftc.teamcode;

/**
 * Created by adam on 9/26/17.
 */
public class MockIMU implements IMU {

    float[] testData;

    int currPoint = 0;

    public MockIMU(float[] mockData) {
        this.testData = mockData;
    }

    @Override
    public float getAngularRotationX() {
        if (currPoint < testData.length) {
            return testData[currPoint];
        }
        return Float.POSITIVE_INFINITY;
    }

    @Override
    public float getAngularRotationY() {
        return 0f;
    }

    @Override
    public float getAngularRotationZ() {
        return 0f;
    }
}
