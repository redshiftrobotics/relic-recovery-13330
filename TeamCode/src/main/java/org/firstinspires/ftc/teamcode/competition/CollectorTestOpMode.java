package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;

/**
 * Created by ariporad on 2017-12-02.
 */

@TeleOp(name="CollectorTest")
public class CollectorTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PulsarRobotHardware r = new PulsarRobotHardware(hardwareMap, null);
        waitForStart();
        while (opModeIsActive()) {
            r.intakeServoLeft.setPosition(1);
            r.intakeServoRight.setPosition(1);
            sleep(1000);
            r.intakeServoLeft.setPosition(0);
            r.intakeServoRight.setPosition(0);
            sleep(1000);
            idle();
        }
    }
}
