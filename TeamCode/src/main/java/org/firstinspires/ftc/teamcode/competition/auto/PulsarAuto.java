package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.blockplacer.Col;
import org.redshiftrobotics.lib.blockplacer.Cryptobox;
import org.redshiftrobotics.lib.blockplacer.Glyph;
import org.redshiftrobotics.lib.debug.DebugHelper;
import org.redshiftrobotics.lib.vuforia.VuforiaController;


abstract public class PulsarAuto extends LinearOpMode {
    private static final boolean JEWEL = false;

    abstract protected Alliance getAlliance();
    abstract protected StartPosition getStartPosition();
    protected boolean isSimpleAuto() { return false; }

    public enum Alliance {
        BLUE, RED;

        public double getFlipFactor() {
            return this == BLUE ? 1 : -1;
        }
        public boolean detectTape(ColorSensor colorSensor) {
            return detectTape(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        }
        public boolean detectTape(int r, int g, int b) {
            return r - b > 50 && this == RED || b - r > 50 && this == BLUE;
        }
    }

    protected enum StartPosition {A, B}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    private Col targetColumn = Col.NONE;
    private Glyph.GlyphColor lastGlyphColor = Glyph.GlyphColor.GRAY; // Always start with gray

    protected PulsarRobotHardware hw;

    private VuforiaController vuforiaController;

    /**
     * IMPORTANT: turning a positive number of degrees is CCW
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing For Auto!");
        telemetry.update();

        hw = new PulsarRobotHardware(this, getAlliance());
        hw.collectorUp();
        hw.jewelsUp(true);
        hw.setFlipperPosition(0);

        vuforiaController = new VuforiaController(hw);

        telemetry.addLine("Ready");
        telemetry.update();


        while (DebugHelper.isEnabled() && !opModeIsActive()) {
            telemetry.addLine("Ready");
            telemetry.addData("Default Col", targetColumn);
            telemetry.update();

            if (gamepad1.a) targetColumn = Col.LEFT;
            if (gamepad1.b) targetColumn = Col.CENTER;
            if (gamepad1.x) targetColumn = Col.RIGHT;
            if (gamepad1.y) targetColumn = Col.NONE;

            idle();
        }

        if (!opModeIsActive()) waitForStart();

        hw.storeCryptoboxTarget();
        hw.setConveyorPower(1);

        scanCryptoKey();
        sleep(1000);
        scanCryptoKey();
        telemetry.addLine("Lowering Jewel");
        telemetry.update();

        if (JEWEL) {
            hw.jewelDown(true);

            knockOffJewel(detectJewel());

            hw.jewelsUp(true);
        }

        if (targetColumn == Col.NONE && getAlliance() == Alliance.BLUE) {
            hw.turn(5, 1000, 0.05);
            scanCryptoKey();
            hw.turn(-5, 1000, 0.05);
        }

        if (targetColumn == Col.NONE) targetColumn = Col.CENTER;

        telemetry.addData("Target Column", targetColumn.toString());
        telemetry.update();

        switch (getStartPosition()) {
            case A:
                autoA();
                break;
            case B:
                autoB();
                break;
        }
    }

    private void autoA() {
        hw.move(-1, 1300);
        hw.strafe(1, 750);
        depositGlyph();
    }

    private void autoB() {
        hw.move(-1, 1500);
        hw.turn(-90, 2000);
        depositGlyph();

        if (!isSimpleAuto()) {
            hw.move(1, 3000);
            collectGlyph();
            collectGlyph();
            hw.move(-1, 3000);
            depositGlyph();
        }
    }

    private void collectGlyph() {
        hw.conveyorOff();
        hw.collectorOn();

        hw.move(1, 750);

        Glyph.GlyphColor color = Glyph.GlyphColor.fromSensor(hw.colorSensors.glyph);

        if (lastGlyphColor == color) {
            hw.collectorOff();
            hw.turn(90, 1000);
            hw.collectorReverse();
            sleep(1000);
            hw.collectorOff();
            hw.turn(-90, 1000);
            collectGlyph();
        } else {
            hw.conveyorOn();
            sleep(2000);
            hw.collectorOff();
            lastGlyphColor = color;
        }
        hw.move(-1, 750);
    }


    private void depositGlyph() {
        long strafeTime;
        if ((getAlliance() == Alliance.BLUE && targetColumn == Col.LEFT) || (getAlliance() == Alliance.RED && targetColumn == Col.RIGHT)) {
            strafeTime = 0;
            // We don't need to do anything, we're already at this column.
        } else if (targetColumn == Col.CENTER) {
            strafeTime = 1000;
        } else {
            strafeTime = 1250;
        }
        if (strafeTime > 0) hw.strafe(1, strafeTime);
        hw.setFlipperPosition(1);
        sleep(1500);
        hw.move(0.2, 1000);
        hw.move(-1, 500, 0);
        hw.move(1, 350);
        hw.strafe(-1, strafeTime);
    }

    private void scanCryptoKey() {
        if (targetColumn != Col.NONE) return;
        targetColumn = vuforiaController.detectColumn();
        targetColumn = vuforiaController.detectColumn();
        telemetry.addData("CryptoKey", targetColumn.toString());
    }

    // Lower jewel before calling this method
    private TargetJewelPosition detectJewel() {
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

        hw.turn(angle, 2000, 0.05);
        scanCryptoKey();
        hw.jewelsUp(true);
        hw.turn(-angle, 2000, 0.05);
    }

    protected TargetJewelPosition getTargetJewel() {
        int red = hw.colorSensors.jewel.red();
        int blue = hw.colorSensors.jewel.blue();

        Alliance backJewel;

        if (red > blue) backJewel = Alliance.RED;
        else if (blue > red) backJewel = Alliance.BLUE;
        else return TargetJewelPosition.NONE;

        if (backJewel == getAlliance()) return TargetJewelPosition.FRONT;
        else return TargetJewelPosition.BACK;
    }
}
