package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.competition.PulsarAuto;

/**
 * Created by ariporad on 2017-11-02.
 */

@Autonomous(name="Jewel Test (Color Sensor)", group="test")
@Disabled
public class JewelDetectorTest extends PulsarAuto {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected StartPosition getStartPosition() {
        return StartPosition.FRONT;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor jewelDetector = hardwareMap.colorSensor.get("leftJewelDetector");
        while (opModeIsActive()) {
            TargetJewelPosition targetJewelPosition = Alliance.BLUE.getTargetJewel(StartPosition.FRONT, jewelDetector);

            telemetry.addData("saw jewelServo", targetJewelPosition.toString());
            telemetry.update();
            idle();
        }
    }
}
