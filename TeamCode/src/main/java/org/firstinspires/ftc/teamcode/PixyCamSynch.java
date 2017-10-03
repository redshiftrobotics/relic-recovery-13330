package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@Autonomous(name = "Sensor: PixySynch", group = "Sensor")
public class PixyCamSynch extends LinearOpMode {
    I2cDeviceSynch pixy;
    byte[] readCache;
    PixyObject pixyObject = new PixyObject();

    int zeroCheck = 0;
    int start = 0;
    boolean found = false;

    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDeviceSynch.get("pixyCam");
        pixy.engage();

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {
            readCache = pixy.read(0x54, 26);

            zeroCheck = 0;
            for(int i = 0; i< 10; i++){
                zeroCheck+=readCache[i];
                if(((readCache[i] < 0) ? readCache[i] + 256 : readCache[i])==85&&((readCache[i+1] < 0) ? readCache[i+1] + 256 : readCache[i+1])==170&&((readCache[i+2] < 0) ? readCache[i+2] + 256 : readCache[i+2])==85&&((readCache[i+3] < 0) ? readCache[i+3] + 256 : readCache[i+3])==170){
                    start = i;
                    found = true;
                }
            }

            if(zeroCheck!=0&&found){
                telemetry.addData("Start", start);
                telemetry.addData("Sync", Integer.toHexString((readCache[start+0] < 0) ? readCache[start+0] + 256 : readCache[start+0]) + Integer.toHexString((readCache[start+1] < 0) ? readCache[start+1] + 256 : readCache[start+1]));
                telemetry.addData("Sync2", Integer.toHexString((readCache[start+2] < 0) ? readCache[start+2] + 256 : readCache[start+2]) + Integer.toHexString((readCache[start+3] < 0) ? readCache[start+3] + 256 : readCache[start+3]));
                telemetry.addData("4", readCache[start+4]);
                telemetry.addData("5", readCache[start+5]);
                telemetry.addData("6", readCache[start+6]);
                telemetry.addData("7", readCache[start+7]);
                telemetry.addData("X Center", (readCache[start+9]==0?0:256) + ((readCache[start+8] < 0) ? readCache[start+8] + 256 : readCache[start+8]));
                telemetry.addData("Y Center", (readCache[start+11]==0?0:256) + ((readCache[start+10] < 0) ? readCache[start+10] + 256 : readCache[start+10]));
                telemetry.addData("Width", (readCache[start+13]==0?0:256) + ((readCache[start+12] < 0) ? readCache[start+12] + 256 : readCache[start+12]));
                telemetry.addData("Width", (readCache[start+15]==0?0:256) + ((readCache[start+14] < 0) ? readCache[start+14] + 256 : readCache[start+14]));

                telemetry.addData("X Center Unit", ((readCache[start+8] < 0) ? readCache[start+8] + 256 : readCache[start+8]));
                telemetry.addData("X Center Carry", readCache[start+9]);
                telemetry.addData("Y Center Unit", ((readCache[start+10] < 0) ? readCache[start+10] + 256 : readCache[start+10]));
                telemetry.addData("Y Center Carry", readCache[start+11]);
                telemetry.addData("Width Unit", readCache[start+12]);
                telemetry.addData("Width Carry", readCache[start+13]);
                telemetry.addData("Height Unit", readCache[start+14]);
                telemetry.addData("Height Carry", readCache[start+15]);
                telemetry.addData(" ", " ");
            }
            found = false;

            telemetry.update();
        }
    }
}