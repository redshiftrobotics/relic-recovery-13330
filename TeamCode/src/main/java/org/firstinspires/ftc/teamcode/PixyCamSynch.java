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

    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");
        pixy.engage();

        waitForStart();

        while (opModeIsActive()) {
            readCache = pixy.read(0x54, 26);

            zeroCheck = 0;
            for(int i = 0; i< 16; i++){
                zeroCheck+=readCache[i];
            }

            if((readCache[1]==-86&&readCache[2]==85)){
                pixyObject.UpdateObject(readCache);
                telemetry.addData("Tracking", "True");
                telemetry.addData("Sig", pixyObject.signature + ", x: " + pixyObject.xCenter + ", y: " + pixyObject.yCenter + ", w: " + pixyObject.width + ", h: " + pixyObject.height);
            }else if(zeroCheck==0){
                telemetry.addData("Sig", pixyObject.signature + ", x: " + pixyObject.xCenter + ", y: " + pixyObject.yCenter + ", w: " + pixyObject.width + ", h: " + pixyObject.height);
                telemetry.addData("Tracking", "False");
            }

            telemetry.update();
        }
    }
}