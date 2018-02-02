package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;

@Autonomous(name = "Encoder Tester", group = "Debug")
@Disabled
public class EncoderTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PulsarRobotHardware hw = new PulsarRobotHardware(this, null);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Encoder Values:");
            telemetry.addData("Front Left",  hw.motors.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", hw.motors.frontRight.getCurrentPosition());
            telemetry.addData("Back Left",   hw.motors.backLeft.getCurrentPosition());
            telemetry.addData("Back Right",  hw.motors.backRight.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}
