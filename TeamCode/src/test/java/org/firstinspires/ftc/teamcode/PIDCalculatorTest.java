package org.firstinspires.ftc.teamcode;


import junit.framework.TestCase;

import org.redshiftrobotics.lib.pid.PIDCalculator;
import org.redshiftrobotics.lib.pid.imu.MockIMU;


/**
 * Created by adam on 9/16/17.
 */


public class PIDCalculatorTest extends TestCase {

    class testCase {
        double expected;
        double input;
        double target;
       testCase expect(double expect) {
           this.expected = expect;
           return this;
       }

        testCase input(double input) {
            this.input = input;
            return this;
        }
        testCase target(double target) {
            this.target = target;
            return this;
        }
    }

    public void testPIDLoop() {
        double[] testIMUData = new double[]{0, 70f, 359, 1};
        testCase[] testCases = new testCase[] {

                //dT = 10

                //Manually calculate P, I, and D each time

                //input angle: 70
                //target 50
                // (70 - 50) + (70-50) * dT + (70 - 50)/(dT)
                // = 20 + 200 + 20/10 = 220 + 2 = 222

                // dT = 10/1000 = 0.01
                // input angle: 70
                // target: 50
                // (70-50) + [(70 - 50) * dT]/2000 + (70-50)/(dT/1000)
                new testCase().target(50).expect(20.0011),

                // input angle: 359
                // target: 1
                // (359 - (360  + 1)) = -2
                // -2  + (-2 * dT) + -2/dT
                // -2 - 20 -0.2 = -22.2
               new testCase().target(1).expect(-2.00011),

                // input angle: 1
                //target: 359
                // (360 + 1) - 359 = 361 - 359 = 2
                // 2 + (2 * dT) + 2/dT = 2 + 20 + 0.2 = +22.2
                new testCase().target(359).expect(2.00011)
        };

        long dTMilliseconds = 10; //assume constant delta T of 10 ms

        MockIMU mockIMU = new MockIMU(testIMUData);
        PIDCalculator controller = new PIDCalculator(mockIMU);
        controller.setTuning(new PIDCalculator.PIDTuning(1, 1, 1));

        for (testCase test : testCases) {
            controller.clearData();
            controller.setTarget(test.target);

            double correction = controller.calculatePID((double)dTMilliseconds/1000);
            System.out.println(correction);
            assertEquals((double) test.expected, correction, 0.1);
        }

    }
}