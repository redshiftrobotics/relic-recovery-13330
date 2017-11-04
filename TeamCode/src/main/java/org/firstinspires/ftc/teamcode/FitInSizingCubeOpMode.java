package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ariporad on 2017-11-04.
 */

@TeleOp(name = "Fit In Sizing Cube")
public class FitInSizingCubeOpMode extends LinearOpMode {
    SizingCubeFitter fitter;

    @Override
    public void runOpMode() throws InterruptedException {
        fitter = new SizingCubeFitter(this);
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Fitting...");
        telemetry.update();
        fitter.fitInSizingCube(true);
        telemetry.addLine("Done!");
        telemetry.update();
    }
}
