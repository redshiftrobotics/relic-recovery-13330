package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Duncan on 10/2/2017.
 */
@Autonomous(name = "Test Auto for Values")
public class PixyCamTestValues  extends LinearOpMode {
    I2cDeviceSynch pixy;
    byte[] readCache;
    PixyObject pixyObject = new PixyObject();
    int frame = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDeviceSynch.get("pixyCam");
        pixy.engage();

        waitForStart();

        while (opModeIsActive()) {
            frame++;
            readCache = pixy.read(0x54, 26);

            System.out.println("Data Set: " + frame);
            for(int i = 0; i < 16; i ++) {
                System.out.println(" - " + i + ": " +  Integer.toHexString(((readCache[i] < 0) ? readCache[i] + 256 : readCache[i])));
            }
            System.out.println("----");
        }
    }
}