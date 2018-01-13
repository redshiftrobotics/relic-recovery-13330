package org.redshiftrobotics.lib.pid;


import junit.framework.TestCase;


public class PIDCalculatorTest extends TestCase {

    class PIDTest {
        double imuData;
        double expected;
        double dT = 10;
        double target;

        PIDTest imuData(double imuData) {
            this.imuData = imuData;
            return this;
        }

        PIDTest expect(double expect) {
            this.expected = expect;
            return this;
        }

        PIDTest target(double target) {
            this.target = target;
            return this;
        }

        PIDTest dT(double dT) {
            this.dT = dT;
            return this;
        }
    }

    public void testPIDLoop() {
        PIDTest[] tests = new PIDTest[] {
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
                new PIDTest().imuData(70).target(50).expect(20.0011),

                // input angle: 359
                // target: 1
                // (359 - (360  + 1)) = -2
                // -2  + (-2 * dT) + -2/dT
                // -2 - 20 -0.2 = -22.2
                new PIDTest().imuData(359).target(1).expect(-2.00011),

                // input angle: 1
                //target: 359
                // (360 + 1) - 359 = 361 - 359 = 2
                // 2 + (2 * dT) + 2/dT = 2 + 20 + 0.2 = +22.2
                new PIDTest().imuData(1).target(359).expect(2.00011)
        };

        MockIMU mockIMU = new MockIMU();
        PIDCalculator controller = new PIDCalculator(mockIMU);
        controller.setTuning(new PIDCalculator.PIDTuning(1, 1, 1));

        for (PIDTest test : tests) {
            mockIMU.setData(test.imuData);
            controller.clearData();
            controller.setTarget(test.target);

            double correction = controller.calculatePID(test.dT / 1000);

            assertEquals(test.expected, correction, 0.1);
        }

    }
}