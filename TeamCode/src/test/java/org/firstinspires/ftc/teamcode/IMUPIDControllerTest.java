package org.firstinspires.ftc.teamcode;


import junit.framework.TestCase;

import org.junit.Test;


/**
 * Created by adam on 9/16/17.
 */
public class IMUPIDControllerTest extends TestCase {
    @Test
    public void testThing() {
        assertEquals(2 + 2, 4);
    }

    @Test
    public void testPIDLoop() {
        float[] testIMUData = new float[]{70f, 60.5f, 59.8f, -14.6f, -5f};
        MockIMU mockIMU = new MockIMU(testIMUData);

    }
}