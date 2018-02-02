package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.debug.DebugHelper;
import org.redshiftrobotics.lib.pid.imu.IMU;
import org.redshiftrobotics.lib.pid.imu.IMUWrapper;

//@TeleOp(name="Pulsar Teleop", group="Pulsar")
//public class PulsarTeleop extends LinearOpMode {
//    private static final boolean USE_PID = true;
//
//    private PulsarRobotHardware hw;
//
//    private IMU hwIMU;
//
//    private static final double DRIVE_POWER_SCALAR = 0.95;
//
//    private double xPower = 0;
//    private double yPower = 0;
//    private double pPower = 0;
//    private double anglePower = 0;
//    private double targetAngle = 180;
//    private double currentAngle = 180;
//    private double cryptoboxAngle = 104;
//    private double lastAngle = 180;
//    private double pConstant = 5;
//
//    private boolean isTurning = false;
//    private boolean lastRB = false;
//    private boolean lastLB = false;
//
//    private double flPower = 0;
//    private double frPower = 0;
//    private double blPower = 0;
//    private double brPower = 0;
//    private double maxPower = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addLine("Initializing for TeleOp!");
//        telemetry.update();
//
//        hw = new PulsarRobotHardware(this, null);
//        hwIMU = new IMUWrapper(hw.hwIMU);
//
//        hw.jewelsUp(true);
//
//        telemetry.addLine("Ready for TeleOp!");
//        telemetry.addLine();
//        telemetry.addLine("Press START to Begin!");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // This order is important
//            readDriverControls(gamepad1);
//            if (USE_PID) fineTuneCurrentAngle(gamepad2);
//            if (USE_PID) computePLoop(); else computeRawMotorPower();
//            driveMotors();
//            if (USE_PID) storeCryptoboxAlignment(gamepad2);
//            sendTelemetry();
//            idle();
//        }
//    }
//
//    /**
//     * Allow fine-tuning of the robot's current target angle.
//     */
//    private void fineTuneCurrentAngle(Gamepad pad) {
//        if (pad.dpad_right != lastRB && pad.dpad_right) { targetAngle += 5; }
//        if (pad.dpad_left != lastLB && pad.dpad_left) { targetAngle -= 5; }
//        lastRB = pad.dpad_right;
//        lastLB = pad.dpad_left;
//    }
//
//    /**
//     * Allow alignment with the cryptobox to be saved and restored.
//     */
//    private void storeCryptoboxAlignment(Gamepad pad) {
//        if (pad.b) cryptoboxAngle = currentAngle;
//        if (pad.x) targetAngle = cryptoboxAngle;
//    }
//
//    /**
//     * Read the driver's controls and store them for later.
//     */
//    private void readDriverControls(Gamepad pad) {
//        xPower = pad.right_stick_x * DRIVE_POWER_SCALAR;
//        yPower = -pad.right_stick_y * DRIVE_POWER_SCALAR;
//        anglePower = -pad.left_stick_x * DRIVE_POWER_SCALAR;
//    }
//
//    /**
//     * Drive the motors based on the result of the P Loop.
//     */
//    private void driveMotors() {
//        hw.frontLeft.setPower(flPower);
//        hw.frontRight.setPower(frPower);
//        hw.backLeft.setPower(blPower);
//        hw.backRight.setPower(brPower);
//    }
//
//    /**
//     * Send Telemetry to the Driver Station.
//     */
//    private void sendTelemetry() {
//        if (!USE_PID) telemetry.addLine("WARNING!!!: Not Using PID");
//        DebugHelper.addLine("Motors:");
//        DebugHelper.addData("Front Left", flPower);
//        DebugHelper.addData("Front Right", frPower);
//        DebugHelper.addData("Back Left", blPower);
//        DebugHelper.addData("Back Right", brPower);
//        DebugHelper.addLine("Input:");
//        DebugHelper.addData("X Power", xPower);
//        DebugHelper.addData("Y Power", yPower);
//        if (USE_PID) {
//            DebugHelper.addLine("PID:");
//            DebugHelper.addData("P Power", pPower);
//            DebugHelper.addData("Angle Power", anglePower);
//            DebugHelper.addData("Current Angle", currentAngle);
//            DebugHelper.addData("Target Angle", targetAngle);
//            DebugHelper.addData("pConstant", pConstant);
//        }
//        telemetry.update();
//    }
//
//    /**
//     * Use a P Loop and stored driver input to move the robot and adjust for rotational error.
//     * Does not actually move the robot, just computes motor powers and stores them.
//     *
//     * @see #driveMotors()
//     */
//    private void computePLoop(){ //Function to control the robot's movements, this includes calculating P and calculating the motor powers
//        lastAngle = currentAngle;
//        currentAngle = hwIMU.getAngularRotationX();
//
//        if (anglePower != 0) isTurning = true;
//        if (Math.abs(lastAngle - currentAngle) < 1 && isTurning) isTurning = false;
//
//        if(isTurning) {
//            targetAngle = currentAngle;
//            pPower = 0.0;
//        } else {
//            if (currentAngle + 360.0 - targetAngle <= 180.0) {
//                pPower = (currentAngle - targetAngle + 360.0) * pConstant;
//            } else if (targetAngle + 360.0 - currentAngle <= 180.0) {
//                pPower = (targetAngle - currentAngle + 360.0) * -1.0 * pConstant;
//            } else if (currentAngle - targetAngle <= 180.0) {
//                pPower = (currentAngle - targetAngle) * pConstant;
//            }
//        }
//
//        pPower = Range.clip(pPower / -180.0, -1.0, 1.0);
//
//        flPower = yPower - xPower + pPower + anglePower;
//        frPower = yPower + xPower - pPower - anglePower;
//        blPower = yPower + xPower + pPower + anglePower;
//        brPower = yPower - xPower - pPower - anglePower;
//
//        maxPower = maxAbs(flPower, frPower, blPower, brPower, 1); // This can't be less than 1
//
//        flPower /= maxPower;
//        frPower /= maxPower;
//        blPower /= maxPower;
//        brPower /= maxPower;
//    }
//
//    /**
//     * If we don't want to use a P Loop, then we can just drive the motors blindly.
//     */
//    private void computeRawMotorPower() {
//        flPower = yPower - xPower;
//        frPower = yPower + xPower;
//        blPower = yPower + xPower;
//        brPower = yPower - xPower;
//    }
//
//    private double maxAbs(double... nums) {
//        double max = 0;
//
//        for (double num: nums) {
//            num = Math.abs(num);
//            if (num > max) max = num;
//        }
//
//        return max;
//    }
//}
