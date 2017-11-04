package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Duncan on 11/3/2017.
 */
@TeleOp(name="LinearServoTest")
public class LinearServoTest extends OpMode{
    Servo conveyorLift;
    @Override
    public void init() {
        conveyorLift = hardwareMap.servo.get("conveyorLift");
    }

    @Override
    public void loop() {
        conveyorLift.setPosition((-gamepad1.left_stick_y));
    }
}
