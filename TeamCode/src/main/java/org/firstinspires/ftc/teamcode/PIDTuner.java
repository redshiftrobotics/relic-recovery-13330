package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Duncan on 10/14/2017.
 */
@TeleOp(name="PID Tuner")
public class PIDTuner extends OpMode {

    BNO055IMU imu;
    IMUPIDController pid;

    long lastTime;

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        pid.setTuning(1, 1, 1);
    }

    @Override
    public void loop() {
        pid.pConst += gamepad1.left_stick_y * (System.currentTimeMillis() - lastTime);
        pid.iConst += gamepad1.right_stick_y * (System.currentTimeMillis() - lastTime);
        if(gamepad1.dpad_down){
            pid.dConst += (System.currentTimeMillis() - lastTime);
        }else if(gamepad1.dpad_up){
            pid.dConst -= (System.currentTimeMillis() - lastTime);
        }
        if(gamepad1.a){
            //Drive Forward
        }
    }
}
