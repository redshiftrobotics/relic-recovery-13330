package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.blockplacer.Col;
import org.redshiftrobotics.lib.blockplacer.Cryptobox;
import org.redshiftrobotics.lib.blockplacer.Glyph;
import org.redshiftrobotics.lib.vuforia.VuforiaController;


abstract public class PulsarAuto extends LinearOpMode {
    private static final boolean JEWEL = true;

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

    protected PulsarRobotHardware hw;

    private Cryptobox cryptobox;

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

        vuforiaController = new VuforiaController(hw);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        hw.storeCryptoboxTarget();
        hw.setConveyorPower(1);

        scanCryptoKey();
        telemetry.addLine("Lowering Jewel");
        telemetry.update();

        if (JEWEL) {
            hw.jewelDown(true);

            knockOffJewel(detectJewel());

            hw.jewelsUp(true);
        }


        // It's a penalty if we cross the center line, so we default to the left as it's the farthest
        // from that. It's also the hardest for TeleOp, so may as well get it over with.
        if (targetColumn == Col.NONE) targetColumn = Col.LEFT;

        telemetry.addData("Target Column", targetColumn.toString());
        telemetry.update();

        if (isSimpleAuto()) {
            switch (getStartPosition()) {
                case A:
                    simpleAutoA();
                    break;
                case B:
                    simpleAutoB();
                    break;
            }
            return;
        }

        // TODO: Do we always want to start with a gray glyph?
        cryptobox = new Cryptobox(Glyph.GlyphColor.GRAY, targetColumn);

        // point to the center line, so that we can drive to the glyph pit
        hw.turn(90, 1000, 0.1);

        hw.collectorDown();

        // off the balancing stone
        hw.move(1, 3000);  // TUNE

        hw.conveyorOn();
        hw.setFlipperPosition(0);

        // point at the glyph pit
        hw.turn(-45, 1000);

        // collect the glyphs
        collectGlyph();
        collectGlyph();

        // Back out of the glyph pit
        hw.move(-1, 1000); // TUNE

        // Go back to the Cryptobox
        hw.turn(45, 1000);
        hw.move(-1, 4000);

        // Move into the general area of the cryptobox
        hw.strafe(1, 1000); // TUNE

        // place the block
        hw.alignWithCryptobox(targetColumn);
        hw.collectorOff();
        hw.setFlipperPosition(1);
        hw.move(0.4, 500); // back up
        hw.move(-1, 500, 0); // ram it in
        hw.move(1, 200); // Park in the safe zone
    }

    private void simpleAutoA() {
        hw.move(-1, 1500);
        hw.strafe(1, 750);
        depositGlyph();
        hw.move(1, 500);
    }

    private void simpleAutoB() {
        hw.move(-1, 1500);
        hw.turn(-90, 2000);
        depositGlyph();
        hw.move(1, 500);
    }

    private void depositGlyph() {
        hw.setFlipperPosition(1);
        sleep(1500);
        hw.move(0.2, 1000);
        hw.move(-1, 500, 0);
    }

    private void scanCryptoKey() {
        if (targetColumn != Col.NONE) return;
        targetColumn = vuforiaController.detectColumn();
        telemetry.addData("CryptoKey", targetColumn.toString());
    }

    private void collectGlyph() {
        hw.conveyorOff();
        hw.collectorOn();

        // move forward a bit, this should collect a glyph
        hw.move(0.5, 500); // TUNE

        hw.collectorOff();

        // Make sure that we could get a cypher with this glyph
        Glyph.GlyphColor color = Glyph.GlyphColor.fromSensor(hw.colorSensors.glyph);
        Col col = cryptobox.getNextBlock(color, true); // dry run

        telemetry.addData("GlyphColor", color.toString());
        telemetry.addData("Col", col.toString());
        telemetry.addData("red",   hw.colorSensors.glyph.red());
        telemetry.addData("blue",  hw.colorSensors.glyph.blue());
        telemetry.addData("green", hw.colorSensors.glyph.green());
        telemetry.addData("alpha", hw.colorSensors.glyph.alpha());
        telemetry.addData("dist",  hw.distanceSensors.glyph.getDistance(DistanceUnit.CM));
        telemetry.update();

        if (col == Col.NONE) {
            hw.turn(90, 1000);
            hw.collectorReverse();
            sleep(200);
            hw.collectorOff();
            hw.turn(-90, 1000);
            collectGlyph();
        } else {
            // Actually add it to our virtual cryptobox.
            targetColumn = cryptobox.getNextBlock(color, false); // do it for real
            hw.conveyorOn();
            hw.collectorOn();
            sleep(350); // TUNE
            hw.collectorOff();
        }
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
