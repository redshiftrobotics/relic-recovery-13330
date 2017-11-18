package org.firstinspires.ftc.teamcode;


import junit.framework.TestCase;

import org.redshiftrobotics.lib.pid.IMUPIDController;


/**
 * Created by adam on 9/16/17.
 */
public class IMUPIDControllerTest extends TestCase {

    class testCase {
        float expected;
        float input;
        float target;
       testCase expect(float expect) {
           this.expected = expect;
           return this;
       }

        testCase input(float input) {
            this.input = input;
            return this;
        }
        testCase target(float target) {
            this.target = target;
            return this;
        }
    }

    public void testThing() {
        assertEquals(2 + 2, 4);
    }

    public void testPIDLoop() {
        float[] testIMUData = new float[]{70f, 359, 1};
        testCase[] testCases = new testCase[] {

                //dT = 10

                //Manually calculate P, I, and D each time

                //input angle: 70
                //target 50
                // (70 - 50) + (70-50) * dT + (70 - 50)/(dT)
                // = 20 + 200 + 20/10 = 220 + 2 = 222
                new testCase().target(50).expect(222.0f),

                // input angle: 359
                // target: 1
                // (359 - (360  + 1)) = -2
                // -2  + (-2 * dT) + -2/dT
                // -2 - 20 -0.2 = -22.2
               new testCase().target(1).expect(-22.2f),

                // input angle: 1
                //target: 359
                // (360 + 1) - 359 = 361 - 359 = 2
                // 2 + (2 * dT) + 2/dT = 2 + 20 + 0.2 = +22.2
                new testCase().target(359).expect(22.2f)
        };

        long dTMilliseconds = 10; //assume constant delta T of 10 ms

        MockIMU mockIMU = new MockIMU(testIMUData);
        IMUPIDController controller = new IMUPIDController(mockIMU);
        controller.setTuning(1f, 1f, 1f);

        for (testCase test : testCases) {
            controller.clearData();
            controller.setTarget(test.target);

            double correction = controller.calculatePID(dTMilliseconds);
            assertEquals((double) test.expected, correction);
        }
    }
}