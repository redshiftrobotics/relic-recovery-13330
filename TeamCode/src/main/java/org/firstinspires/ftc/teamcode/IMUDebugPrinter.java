package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ariporad on 2017-11-25.
 */

@TeleOp(name = "imudebug")
public class IMUDebugPrinter extends OpMode {
    BNO055IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    @Override
    public void loop() {
        telemetry.addData("Angle: ", imu.getAngularOrientation().firstAngle );
        telemetry.update();
    }
}
