package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.Vector2D;
import org.redshiftrobotics.lib.blockplacer.Col;
import org.redshiftrobotics.lib.blockplacer.Cryptobox;
import org.redshiftrobotics.lib.blockplacer.Glyph;
import org.redshiftrobotics.lib.pid.StraightPIDController;
import org.redshiftrobotics.lib.vuforia.VuforiaController;
import org.redshiftrobotics.lib.pid.TurningPIDController;


abstract public class PulsarAuto extends LinearOpMode {
    private static final boolean JEWEL = false;
    private static final boolean COLLECT = true;

    private static final double X_SCALAR = -1;
    private static final double Y_SCALAR = 1;
    private static final double ANGLE_SCALAR = -1;

    private static final double FORWARD_SPEED_SCALAR = 0.9;
    private static final double STRAFE_SPEED_SCALAR = 0.9;

    private static final long TWEEN_TIME = 500;

    abstract protected Alliance getAlliance();
    abstract protected StartPosition getStartPosition();
    protected boolean isSimpleAuto() { return false; }

    public enum Alliance {
        BLUE, RED;

        public double getFlipFactor() {
            return this == BLUE ? 1 : -1;
        }
        public boolean detectTape(int r, int g, int b) { return this == BLUE ? b > r : r > b; }
        public boolean detectCryptoBoxDevider(int r, int g, int b) {
            if(this == BLUE) {
                return b > 200 && r < 150 && g < 150;
            } else {
                return r > 200 && b < 150 && g < 150;
            }
        }
    }

    protected enum StartPosition {BACK, FRONT}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    private Col targetColumn = Col.NONE;

    protected PulsarRobotHardware hw;

    private Cryptobox cryptobox;

    private VuforiaController vuforiaController;

    /**
     * IMPORTANT:
     * In auto, we drive the robot with the flipper forwards (ie. positive speed). This is the opposite
     * of how we do it in TeleOp.
     *
     *
     * Additionally, we mirror all X movements and all turning when we are on the red alliance. (We
     * also do some other magic, such as some minor speed scaling.) This allows us to write just one
     * auto, and have it automagically work on both sides. Because of this, **NEVER USE THE PID
     * CONTROLLERS DIRECTLY** if you do, nothing will work. Use #move/#strafe/#turn.
     *
     * Therefore, here is a motion cheatsheet:
     * move(+): move towards the flipper
     * move(-): move towards the collector
     * strafe(+): move towards the center line
     * strafe(-): move away towards the wall/Alliance Station
     * turn(+): turn towards the center line (CW for blue, CCW for red)
     * turn(-): turn away from the center line (CCW for blue, CW for red)
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

        scanCryptoKey();
        telemetry.addLine("Lowering Jewel");
        telemetry.update();

        if (JEWEL) {
            hw.jewelDown(true);

            knockOffJewel(detectJewel());

            hw.jewelsUp(true);

        }

        if (isSimpleAuto()) return;

        // It's a penalty if we cross the center line, so we default to the left as it's the farthest
        // from that. It's also the hardest for TeleOp, so may as well get it over with.
        if (targetColumn == Col.NONE) targetColumn = Col.LEFT;

        telemetry.addData("Target Column", targetColumn.toString());
        telemetry.update();

        // TODO: Do we always want to start with a gray glyph?
        cryptobox = new Cryptobox(Glyph.GlyphColor.GRAY, targetColumn);

        // point to the center line, so that we can drive to the glyph pit
        turn(-90, 1000, 0.2);

        hw.collectorDown();

        // off the balancing stone
        move(1, 3000);  // TUNE

        hw.motors.conveyor.setPower(1);
        hw.setFlipperPosition(0);

        // point at the glyph pit
        turn(45, 1000);

        // collect the glyphs
        collectGlyph();
        collectGlyph();

        // Back out of the glyph pit
        move(-1, 1000); // TUNE

        // Go back to the Cryptobox
        turn(-45, 1000);
        move(1, 4000);

        switch (targetColumn) {
            case LEFT:
                strafe(-1, 3000);
                break;
            case CENTER:
                strafe(-1, 2000);
                break;
            case RIGHT:
                strafe(-1, 1000);
                break;
            default: throw new IllegalStateException("unknown targetColumn");
        }

        // place the block
        hw.collectorOff();
        hw.setFlipperPosition(1);
        // TODO: strafe to column, place
    }

    /**
     * This method assumes that the front center of our robot is within the cryptobox triangle.
     */
    private void alignWithCryptobox() {

    }

    private void scanCryptoKey() {
        if (targetColumn != Col.NONE) return;
        targetColumn = vuforiaController.detectColumn();
        telemetry.addData("CryptoKey", targetColumn.toString());
    }

    private void collectGlyph() {
        // Don't collect yet
        hw.collectorOff();

        // Till we're touching the glyph
        // This is disabled for now, because we're trying to collect blindly
        //while (hw.glyphDetector.getDistance(DistanceUnit.CM) > 1) { // TUNE
        //    move(1, 10); // move a tiny amount
        //}

        // Make sure that we could get a cypher with this glyph
        Glyph.GlyphColor color = (hw.colorSensors.glyph.red() > hw.colorSensors.glyph.blue() && hw.colorSensors.glyph.red() > 150) ? Glyph.GlyphColor.BROWN : Glyph.GlyphColor.GRAY;
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
            throw new IllegalStateException("Couldn't figure out where to place glyph"); // FIXME
        }

        // Actually collect it
        hw.collectorOn();

        // Drive forward a very small amount
        move(-1, 1000); // TUNE
        move(1, 500); // TUNE

        hw.collectorOff();

        // Actually add it to our virtual cryptobox.
        targetColumn = cryptobox.getNextBlock(color, false); // do it for real
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

        turn(angle, 2000, 0.2);
        scanCryptoKey();
        hw.jewelsUp(true);
        turn(-angle, 2000, 0.2);
    }

    protected void depositGlyph() {
        hw.setFlipperPosition(1);
        sleep(5000); // TUNE

        // Push in it
        move(-1, 500); // TUNE
        move(1.0, 500, 0); // TUNE
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

    /**
     * Helpers so that we can easily mirror for the other alliance
     */
    // We negate speed because we drive the robot backwards for auto
    private void move(double speed, long time) {
        move(speed, time, TWEEN_TIME);
    }
    private void move(double speed, long time, long tweenTime) {
        hw.straightPIDController.move(-speed * FORWARD_SPEED_SCALAR, time, tweenTime);
    }
    // This method doesn't have a tweenTime-less variant because move(num, num, num) is ambiguous.
    private void move(long time, long tweenTime, double xPower, double yPower) {
        move(time, tweenTime, new Vector2D(xPower, yPower));
    }
    private void move(long time, Vector2D velocity) {
        move(time, TWEEN_TIME, velocity);
    }
    private void move(long time, long tweenTime, Vector2D velocity) {
        double x = X_SCALAR * velocity.getXComponent() * FORWARD_SPEED_SCALAR;
        double y = Y_SCALAR * velocity.getYComponent() * getAlliance().getFlipFactor() * STRAFE_SPEED_SCALAR;
        hw.straightPIDController.move(time, tweenTime, new Vector2D(x, y));
    }

    private void strafe(double speed, long time) {
        strafe(speed, time, TWEEN_TIME);
    }
    private void strafe(double speed, long time, long tweenTime) {
        hw.straightPIDController.strafe(speed * Y_SCALAR * getAlliance().getFlipFactor() * STRAFE_SPEED_SCALAR, time, tweenTime);
    }

    // We negate angle because we drive the robot backwards for auto
    private void turn(double angle, long time) {
        hw.turningPIDController.turn(angle * ANGLE_SCALAR * getAlliance().getFlipFactor(), time);
    }
    private void turn(double angle, long time, double powerConstant) {
        hw.turningPIDController.turn(angle * ANGLE_SCALAR * getAlliance().getFlipFactor(), time, powerConstant);
    }
}
