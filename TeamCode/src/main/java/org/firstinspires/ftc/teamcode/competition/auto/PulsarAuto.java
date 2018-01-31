package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.pid.StrafePIDController;
import org.redshiftrobotics.lib.vuforia.VuforiaController;
import org.redshiftrobotics.lib.pid.ForwardPIDController;
import org.redshiftrobotics.lib.pid.TurningPIDController;


abstract public class PulsarAuto extends LinearOpMode {
    private static final boolean JEWEL = false;

    abstract protected Alliance getAlliance();
    abstract protected StartPosition getStartPosition();
    protected boolean isSimpleAuto() { return false; }

    public enum Alliance {
        BLUE, RED;

        protected double getScalingFactor() {
            return this == BLUE ? 1 : -1;
        }
    }

    protected enum StartPosition {BACK, FRONT}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    private StartPosition startPosition = getStartPosition();
    private PulsarAuto.Alliance alliance = getAlliance();

    private RelicRecoveryVuMark targetColumn;
    private VuforiaController vuforiaController;
    protected PulsarRobotHardware hw;
    protected ForwardPIDController forwardPIDController;
    protected TurningPIDController turningPIDController;
    protected StrafePIDController strafePIDController;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing For Auto!");
        telemetry.update();

        hw = new PulsarRobotHardware(this, getAlliance());
        hw.collectorUp();
        hw.jewelsUp(true);

        vuforiaController = new VuforiaController(hw);

        forwardPIDController = new ForwardPIDController(hw);
        turningPIDController = new TurningPIDController(hw);
        strafePIDController = new StrafePIDController(hw);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        targetColumn = vuforiaController.detectColumn();
        telemetry.addData("CryptoKey", targetColumn.toString());
        telemetry.addLine("Lowering Jewel");
        telemetry.update();

        if (JEWEL) {
            hw.jewelDown(true);

            knockOffJewel(detectJewel());

            hw.jewelsUp(true);

        }

        // XXX: done to here
        if (!isSimpleAuto()) {
            scoreInCryptobox(targetColumn);
        }

        while (opModeIsActive()) { idle(); } // This prevents the servos from ragdolling.
    }

    // Lower jewel before calling this method
    private TargetJewelPosition detectJewel() throws InterruptedException {
        TargetJewelPosition targetJewelPosition = getTargetJewel();

        telemetry.addData("Back Jewel", targetJewelPosition.toString());

        if (targetJewelPosition == TargetJewelPosition.NONE) {
            telemetry.addLine("Couldn't see jewel, moving to alternate position...");
            telemetry.update();

            hw.jewelMoveAlt(true);

            targetJewelPosition = getTargetJewel();
        }

        return targetJewelPosition;
    }

    private void knockOffJewel(TargetJewelPosition targetJewelPosition) {
        if (targetJewelPosition == TargetJewelPosition.NONE) return;

        double angle = targetJewelPosition == TargetJewelPosition.FRONT ? -10 : 10;

        turningPIDController.turn(angle, 2000, 0.2);

        if (targetColumn == RelicRecoveryVuMark.UNKNOWN) {
            targetColumn = vuforiaController.detectColumn();
            telemetry.addData("CryptoKey (Take 2)", targetColumn.toString());
            telemetry.update();
        }
        hw.jewelsUp(true);
        turningPIDController.turn(-angle, 2000, 0.2);
    }

    private void scoreInCryptobox(RelicRecoveryVuMark column) throws InterruptedException {
        hw.collectorDown();
        forward(1, 1575);
        strafe(1, 1010);
        hw.setFlipperPosition(1);
        sleep(1000);
        forward(-1, 500);
        forward(1, 500);
        forward(-1, 500);
    }

    protected TargetJewelPosition getTargetJewel() {
        int red = hw.jewelDetector.red();
        int blue = hw.jewelDetector.blue();

        Alliance backJewel;

        if (red > blue) backJewel = Alliance.RED;
        else if (blue > red) backJewel = Alliance.BLUE;
        else return TargetJewelPosition.NONE;

        if (backJewel == getAlliance()) return TargetJewelPosition.FRONT;
        else return TargetJewelPosition.BACK;
    }

    /**
     * Helpers so that we can easily mirror for the other alliance
     */
    // These two don't actually mirror anything, they're just for consistency.
    private void forward(double speed, long time) {
        forwardPIDController.move(speed, time);
    }
    private void forward(double speed, long time, long tweenTime) {
        forwardPIDController.move(speed, time, tweenTime);
    }

    private void strafe(double speed, long time) {
        strafePIDController.move(speed * getAlliance().getScalingFactor(), time);
    }
    private void strafe(double speed, long time, long tweenTime) {
        strafePIDController.move(speed * getAlliance().getScalingFactor(), time, tweenTime);
    }

    private void turn(double angle, long time) {
        turningPIDController.turn(angle, time);
    }
    private void turn(double angle, long time, double powerConstant) {
        turningPIDController.turn(angle, time, powerConstant);
    }
}
