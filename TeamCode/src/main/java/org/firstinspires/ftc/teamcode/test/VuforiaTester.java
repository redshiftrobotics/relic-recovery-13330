package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.vuforia.VuforiaController;

@Autonomous(name="Vuforia Tester", group = "Debug")
public class VuforiaTester extends LinearOpMode {
    public void runOpMode() {
        PulsarRobotHardware hw = new PulsarRobotHardware(this, null);
        VuforiaController vc = new VuforiaController(hw);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Welcome to the Vuforia Tester!");
            telemetry.addLine();
            telemetry.addData("Detected Col", vc.detectColumn().toString());
        }
    }
}
