package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

/**
 * Created by Duncan on 1/20/2018.
 */

@TeleOp(name = "Duncan's dumb test")
public class ServoAllTest extends LinearOpMode{

    List<Servo> servos;
    int index = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servos = hardwareMap.getAll(Servo.class);
        waitForStart();
        while(opModeIsActive()){

        }
    }
}
