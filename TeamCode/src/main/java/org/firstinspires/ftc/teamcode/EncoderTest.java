package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.redshiftrobotics.lib.DcMotorEncoder;
import org.redshiftrobotics.lib.EncoderPositioner;

@TeleOp(name = "Encoder Test")
public class EncoderTest extends OpMode {
    EncoderPositioner positioner;

    @Override
    public void init() {
        positioner = new EncoderPositioner(
                new DcMotorEncoder(hardwareMap.dcMotor.get("x")),
                new DcMotorEncoder(hardwareMap.dcMotor.get("y")),
                ((7.125 * 2 * Math.PI) / 1300) // Not quite right
        );
    }

    @Override
    public void start() {
        super.start();
        positioner.reset();
    }

    @Override
    public void stop() {
        super.stop();
        positioner.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Position", positioner.getPosition());
        telemetry.update();
    }
}

