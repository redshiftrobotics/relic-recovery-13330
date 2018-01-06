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
            telemetry.addData("Front Left", hw.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", hw.frontRight.getCurrentPosition());
            telemetry.addData("Back Left", hw.backLeft.getCurrentPosition());
            telemetry.addData("Back Right", hw.backRight.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}
