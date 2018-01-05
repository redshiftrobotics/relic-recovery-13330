package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;

/**
 * Created by Duncan on 11/15/2017.
 */
@TeleOp(name="Pulsar Teleop", group="Pulsar")
public class PulsarTeleop extends LinearOpMode{
    private PulsarRobotHardware hw;

    private static final double CONVEYOR_BELT_POWER_SCALAR = 0.6;

    private static final double INTAKE_POWER_SCALAR = 0.5;
    private static final double INTAKE_TURNING_SCALAR = 0.1;

    private static final double DRIVE_POWER_SCALAR = 0.95;

    private static final double COLLECT_TIME = 3000;

    private double xPower = 0;
    private double yPower = 0;
    private double pPower = 0;
    private double anglePower = 0;
    private double targetAngle = 180;
    private double currentAngle = 180;
    private double cryptoboxAngle = 104;
    private double lastAngle = 180;
    private double pConstant = 5;

    private boolean isTurning = false;
    private boolean lastRB = false;
    private boolean lastLB = false;

    private double collectStartTime = 0;
    private boolean isCollecting = false;
    private boolean lastBack = false;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;
    private double maxPower = 0;

    private String fuckingDebug = "nothing";

    // For some bizarre reason, using a normal OpMode breaks everything. So we use a LinearOpMode.
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        telemetry.addLine("Initializing for TeleOp!");
        telemetry.update();

        hw = new PulsarRobotHardware(hardwareMap, null);

        hw.initializePositionsTeleop();
        hw.conveyorDown();

        telemetry.addLine("Ready for TeleOp!");
        telemetry.addLine("IT WORKS");
        telemetry.addLine();
        telemetry.addLine("Press START to Begin!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            readDriverControls(gamepad1);
            fineTuneCurrentAngle(gamepad2);
            //debug_updatePConstant(gamepad2);
            computePLoop();
            driveMotors();
            controlIntake(gamepad2);
            controlConveyor(gamepad2);
            controlIntakeShortcut(gamepad2);
            storeCryptoboxAlignment(gamepad2);
            sendTelemetry();
            idle();
        }
    }

    private void controlIntakeShortcut(Gamepad pad) {
        if (pad.dpad_up != lastBack && pad.dpad_up && !isCollecting) {
            telemetry.addLine("Collecting...");
            isCollecting = true;
            hw.intakeUp();
            hw.conveyor.setPower(hw.CONVEYOR_SPEED);
            collectStartTime = System.currentTimeMillis();
        }

        if (isCollecting && System.currentTimeMillis() - collectStartTime >= COLLECT_TIME) {
            telemetry.addLine("Done Collecting");
            hw.conveyor.setPower(0);
            hw.intakeDown();
            collectStartTime = 0;
            isCollecting = false;
        }

        lastBack = pad.dpad_up;
    }

    private void fineTuneCurrentAngle(Gamepad pad) {
        if (pad.dpad_right != lastRB && pad.dpad_right) { targetAngle += 5; }
        if (pad.dpad_left != lastLB && pad.dpad_left) { targetAngle -= 5; }
        lastRB = pad.dpad_right;
        lastLB = pad.dpad_left;
    }

    private void storeCryptoboxAlignment(Gamepad pad) { //Function that updates the stored rotation, can be customized to use a specific controller
        if (pad.b) cryptoboxAngle = currentAngle;
        if (pad.x) targetAngle = cryptoboxAngle;
    }

    private void readDriverControls(Gamepad pad){ //Function that updates the movement constants, can be customized to use a specific controller
        xPower = pad.right_stick_x * DRIVE_POWER_SCALAR;
        yPower = -pad.right_stick_y * DRIVE_POWER_SCALAR;
        anglePower = -pad.left_stick_x * DRIVE_POWER_SCALAR;
    }

    private void controlIntake(Gamepad pad) {
        // Intake Power
        hw.leftIntake.setPower((-pad.right_stick_y * INTAKE_POWER_SCALAR) - (pad.right_stick_x * INTAKE_TURNING_SCALAR));
        hw.rightIntake.setPower((-pad.right_stick_y * INTAKE_POWER_SCALAR) + (pad.right_stick_x * INTAKE_TURNING_SCALAR));

        // Intake Positioning
        if (pad.y) { hw.intakeUp(); }
        if (pad.a) { hw.intakeDown(); }
    }

    private void controlConveyor(Gamepad pad) {
        // Conveyor Power
        hw.conveyor.setPower(-pad.left_stick_y * CONVEYOR_BELT_POWER_SCALAR);

        // Conveyor Positioning
        //if (pad.dpad_up) { hw.conveyorUp(); }
        //if (pad.dpad_down) { hw.conveyorDown(); }
    }

    private void driveMotors(){ //Function to control all hardware devices on the robot
        hw.frontLeft.setPower(flPower);
        hw.frontRight.setPower(frPower);
        hw.backLeft.setPower(blPower);
        hw.backRight.setPower(brPower);
    }

    private void sendTelemetry(){ //Function to telemetry out important values
        telemetry.addData("Current Angle", currentAngle + ", Target Angle: " + targetAngle);
        telemetry.addData("debug", fuckingDebug);
        telemetry.addData("pConstant", pConstant);
        telemetry.addData("FL", flPower);
        telemetry.addData("FR", frPower);
        telemetry.addData("BL", blPower);
        telemetry.addData("BR", brPower);
        telemetry.addData("XP", xPower);
        telemetry.addData("YP", yPower);
        telemetry.addData("PP", pPower);
        telemetry.addData("AP", anglePower);
        telemetry.update();
    }

    private void computePLoop(){ //Function to control the robot's movements, this includes calculating P and calculating the motor powers
        lastAngle = currentAngle;
        currentAngle = hw.imu.getAngularOrientation().firstAngle + 180f;
        if (anglePower != 0) isTurning = true;
        if (Math.abs(lastAngle - currentAngle) < 1 && isTurning) isTurning = false;
        if(isTurning){
            targetAngle=currentAngle;
            pPower = 0f;
        }else {
            if (currentAngle + 360f - targetAngle <= 180f) {
                pPower = (currentAngle - targetAngle + 360f) * pConstant;
                fuckingDebug = "first";
            } else if (targetAngle + 360f - currentAngle <= 180f) {
                pPower = (targetAngle - currentAngle + 360f) * -1f * pConstant;
                fuckingDebug = "second";
            } else if (currentAngle - targetAngle <= 180f) {
                pPower = (currentAngle - targetAngle) * pConstant;
                fuckingDebug = "third";
            } else {
                pPower = 7;
                fuckingDebug = "pizza";
            }
        }
        pPower = Range.clip(pPower / -180f, -1, 1);

        flPower = (yPower - xPower + pPower + anglePower);
        frPower = (yPower + xPower - pPower - anglePower);
        blPower = (yPower + xPower + pPower + anglePower);
        brPower = (yPower - xPower - pPower - anglePower);

        maxPower = Math.abs(flPower);
        if(Math.abs(frPower)>maxPower) maxPower=Math.abs(frPower);
        if(Math.abs(blPower)>maxPower) maxPower=Math.abs(blPower);
        if(Math.abs(brPower)>maxPower) maxPower=Math.abs(brPower);
        if(maxPower<1) maxPower=1;

        flPower /= maxPower;
        frPower /= maxPower;
        blPower /= maxPower;
        brPower /= maxPower;
    }
}
