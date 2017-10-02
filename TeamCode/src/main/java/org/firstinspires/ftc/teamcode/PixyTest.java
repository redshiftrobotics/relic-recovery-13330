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
        telemetry.addData("Name:", pixyCam.getDeviceName());
        telemetry.update();
    }
}
