package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by adam on 10/10/17.
 */

@TeleOp(name="Measure Encoders")

public class EncoderRotationMeasure extends OpMode {

    public DcMotor motorX;
    public DcMotor motorY;
    @Override
    public void init() {
        motorX = hardwareMap.dcMotor.get("x");
        motorX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorY = hardwareMap.dcMotor.get("y");
        motorY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("motor distance (x, y): ", "(" +  motorX.getCurrentPosition() + ", " + motorY.getCurrentPosition() + ")");
        telemetry.update();
    }
}
