package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;


@TeleOp(name = "Fit In Cube")
public class FitInCube extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PulsarRobotHardware hw = new PulsarRobotHardware(this, PulsarAuto.Alliance.RED);
        waitForStart();
        hw.jewelsUp(true);
        hw.collectorUp();
    }
}
