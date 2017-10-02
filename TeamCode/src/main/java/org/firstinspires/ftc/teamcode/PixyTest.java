package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Duncan on 10/1/2017.
 */

public class PixyTest extends OpMode {

    PixyCam pixyCam;

    @Override
    public void init() {
        pixyCam = hardwareMap.get(PixyCam.class, "pixyCam");
    }

    @Override
    public void loop() {
        pixyCam.updateData();
        telemetry.addData("Name:", pixyCam.getDeviceName());
        telemetry.addData("Sync", pixyCam.data.sync);
        telemetry.addData("Signature", pixyCam.data.signature);
        telemetry.addData("X Center", pixyCam.data.xCenter);
        telemetry.addData("Y Center", pixyCam.data.yCenter);
        telemetry.addData("Width", pixyCam.data.width);
        telemetry.addData("Height", pixyCam.data.height);
        telemetry.update();
    }
}
