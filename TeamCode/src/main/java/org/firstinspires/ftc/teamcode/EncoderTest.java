package org.firstinspires.ftc.teamcode;

import org.redshiftrobotics.lib.AutoOpMode;

/**
 * Created by ariporad on 2017-10-18.
 */

public class EncoderTest extends AutoOpMode {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        telemetry.addData("position", this.positioner.getPosition());
        telemetry.update();
    }
}
