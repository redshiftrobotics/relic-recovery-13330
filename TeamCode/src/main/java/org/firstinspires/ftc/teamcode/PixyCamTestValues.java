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
    String value;
    int references[] = new int[16];
    int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDeviceSynch.get("pixyCam");
        pixy.engage();

        waitForStart();

        while (opModeIsActive()) {
            frame++;
            count = 0;
            readCache = pixy.read(0x00, 26);

            for(int i = 0; i < 25; i ++){
                value += Integer.toHexString(((readCache[i] < 0) ? readCache[i] + 256 : readCache[i])) + ", ";
            }

            System.out.println(frame + ": (" + value + ")");
            value = "";

            for(int i = 0; i < 256; i ++){
                if(pixy.read8(i)==85){
                    if(pixy.read8(i+2)==85){
                        references[count] = i;
                        count++;
                    }
                }
            }
            for(int i = 0; i < 16; i++){
                telemetry.addData(references[i] + "", "x: " + ((readCache[references[i]+9]==0?0:256) + ((readCache[references[i]+8] < 0) ? readCache[references[i]+8] + 256 : readCache[references[i]+8])) + ", y: " + ((readCache[references[i]+11]==0?0:256) + ((readCache[references[i]+10] < 0) ? readCache[references[i]+10] + 256 : readCache[references[i]+10])) + ", w: " + ((readCache[references[i]+13]==0?0:256) + ((readCache[references[i]+12] < 0) ? readCache[references[i]+12] + 256 : readCache[references[i]+12])) + ", h: " + ((readCache[references[i]+15]==0?0:256) + ((readCache[references[i]+14] < 0) ? readCache[references[i]+14] + 256 : readCache[references[i]+14])));
            }
            telemetry.addData("", "");
            telemetry.update();
        }
    }
}