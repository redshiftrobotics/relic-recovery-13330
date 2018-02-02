package org.firstinspires.ftc.teamcode.lib;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;
import org.redshiftrobotics.lib.RobotHardware;
import org.redshiftrobotics.lib.debug.DebugHelper;
import org.redshiftrobotics.lib.pid.PIDCalculator;
import org.redshiftrobotics.lib.pid.StraightPIDController;
import org.redshiftrobotics.lib.pid.TurningPIDController;
import org.redshiftrobotics.lib.pid.imu.IMU;
import org.redshiftrobotics.lib.pid.imu.IMUWrapper;

public class PulsarRobotHardware implements RobotHardware {
    public class Servos {
        public final Servo leftJewel;
        public final Servo rightJewel;
        public final Servo jewel;

        public final Servo leftFlipper;
        public final Servo rightFlipper;

        public final Servo leftCollection;
        public final Servo rightCollection;

        protected Servos(HardwareMap hardwareMap) {
            leftJewel = hardwareMap.servo.get("r1s3");
            leftJewel.setDirection(Servo.Direction.REVERSE);
            rightJewel = hardwareMap.servo.get("r1s2");
            rightJewel.setDirection(Servo.Direction.REVERSE);
            jewel = alliance == PulsarAuto.Alliance.BLUE ? leftJewel : rightJewel;


            leftFlipper = hardwareMap.servo.get("r2s1");
            leftFlipper.setDirection(Servo.Direction.REVERSE);
            rightFlipper = hardwareMap.servo.get("r2s2");

            leftCollection = hardwareMap.servo.get("r1s1");
            leftCollection.setDirection(Servo.Direction.REVERSE);
            rightCollection = hardwareMap.servo.get("r1s0");
            rightCollection.setDirection(Servo.Direction.REVERSE);
        }
    }

    public class Motors {
        public final DcMotor frontLeft;
        public final DcMotor frontRight;
        public final DcMotor backLeft;
        public final DcMotor backRight;

        public final DcMotor relic;
        public final DcMotor conveyor;

        public final DcMotor leftCollection;
        public final DcMotor rightCollection;

        protected Motors(HardwareMap hardwareMap) {
            frontLeft = hardwareMap.dcMotor.get("r1m0");
            frontRight = hardwareMap.dcMotor.get("r1m1");
            backLeft = hardwareMap.dcMotor.get("r1m2");
            backRight = hardwareMap.dcMotor.get("r1m3");
            backRight.setDirection(DcMotorSimple.Direction.REVERSE); // XXX: Why only backRight?

            relic = hardwareMap.dcMotor.get("r2m0");
            conveyor = hardwareMap.dcMotor.get("r2m1");

            leftCollection = hardwareMap.dcMotor.get("r2m2");
            leftCollection.setDirection(DcMotorSimple.Direction.REVERSE);
            rightCollection = hardwareMap.dcMotor.get("r2m3");
            rightCollection.setDirection(DcMotorSimple.Direction.REVERSE);

        }
    }

    public class ColorSensors {
        public final ColorSensor leftJewel;
        public final ColorSensor rightJewel;
        public final ColorSensor jewel;

        public final ColorSensor tape;
        public final ColorSensor glyph;
        public final ColorSensor cryptobox;

        protected ColorSensors(HardwareMap hardwareMap) {
            leftJewel = hardwareMap.colorSensor.get("r1c1");
            rightJewel = hardwareMap.colorSensor.get("r1c2");
            jewel = alliance == PulsarAuto.Alliance.BLUE ? leftJewel : rightJewel;

            tape = hardwareMap.get(ColorSensor.class, "r2c2");
            glyph = hardwareMap.get(ColorSensor.class, "r1c0");
            cryptobox = hardwareMap.get(ColorSensor.class, "r2c1");

        }
    }

    public class DistanceSensors {
        public final DistanceSensor glyphNear;
        public final DistanceSensor glyphFar;
        public final DistanceSensor glyph;

        public final DistanceSensor cryptobox;

        protected DistanceSensors(HardwareMap hardwareMap) {
            glyphFar = hardwareMap.get(DistanceSensor.class, "r1c0");
            //glyphNear = hardwareMap.get(DistanceSensor.class, "r2r2");
            //glyph = new ComboDistanceSensor(glyphNear, glyphFar);
            glyphNear = glyphFar;
            glyph = glyphNear;

            cryptobox = hardwareMap.get(DistanceSensor.class, "r2c1");

        }
    }

    // At the start of auto, when we can be reasonably certain that we're parallel to the wall, we
    // store our IMU target, so that TeleOp can use it later to align with the cryptobox.
    private static double cryptoboxTarget;

    public final PulsarAuto.Alliance alliance;

    public final LinearOpMode opMode;
    public final Context appContext;
    public final Telemetry telemetry;

    private final HardwareMap hardwareMap;

    public final Servos servos;
    public final Motors motors;
    public final ColorSensors colorSensors;
    public final DistanceSensors distanceSensors;


    public final IMU imu;
    public final BNO055IMU hwIMU;


    // XXX: I'm not sure about this
    public final StraightPIDController straightPIDController;
    public final TurningPIDController turningPIDController;

    // Constants
    public final double LEFT_JEWEL_UP_POSITON = 0.6;
    public final double LEFT_JEWEL_DOWN_POSITON = 0;
    public final double LEFT_JEWEL_ALT_DOWN_POSITON = 0.05;
    public final double RIGHT_JEWEL_UP_POSITON = 0.5;
    public final double RIGHT_JEWEL_DOWN_POSITON = 0.8;
    public final double RIGHT_JEWEL_ALT_DOWN_POSITON = 0.75;

    public final double CONVEYOR_SPEED = 0.65;
    public final double FLIPPER_POSITION_SCALAR = 0.8;
    public final double FLIPPER_MIN_POSITION = 0.01;

    public final double COLLECTION_UP_POSITION = 1;
    public final double COLLECTION_DOWN_POSITION = 0;

    // FIXME: dividing all corrections by 2000 to prevent overflow is bad
    public final double CORRECTION_SCALAR = 1.0 / 2000.0;


    public PulsarRobotHardware(LinearOpMode opMode, PulsarAuto.Alliance alliance) {
        this.alliance = alliance;
        this.opMode = opMode;

        telemetry = opMode.telemetry;
        DebugHelper.setTelemetry(telemetry);

        hardwareMap = opMode.hardwareMap;
        appContext = opMode.hardwareMap.appContext;

        hwIMU = hardwareMap.get(BNO055IMU.class, "hwIMU");
        initalizeIMU(hwIMU);
        imu = new IMUWrapper(hwIMU);

        straightPIDController = new StraightPIDController(this);
        turningPIDController = new TurningPIDController(this);

        servos = new Servos(hardwareMap);
        motors = new Motors(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
    }

    private void initalizeIMU(BNO055IMU imu) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUConfig.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu.initialize(parameters);
    }

    public void initializePositionsAuto() {
        collectorUp();
        jewelsUp(false);

        telemetry.addLine("Servo Positions: initialized");
    }

    public void initializePositionsTeleop() {
        jewelsUp(true);
    }


    public void setFlipperPosition(double position) {
        position *= FLIPPER_POSITION_SCALAR;
        if (position < FLIPPER_MIN_POSITION) position = FLIPPER_MIN_POSITION;
        servos.leftFlipper.setPosition(position);
        servos.rightFlipper.setPosition(position);
    }

    public void jewelsUp(boolean sleep) {
        servos.leftJewel.setPosition(LEFT_JEWEL_UP_POSITON);
        servos.rightJewel.setPosition(RIGHT_JEWEL_UP_POSITON);
        if (sleep) opMode.sleep(200);
    }
    public void jewelDown(boolean sleep) {
        servos.jewel.setPosition(alliance == PulsarAuto.Alliance.BLUE ? LEFT_JEWEL_DOWN_POSITON : RIGHT_JEWEL_DOWN_POSITON);
        if (sleep) opMode.sleep(200);
    }
    public void jewelMoveAlt(boolean sleep) {
        servos.jewel.setPosition(alliance == PulsarAuto.Alliance.BLUE ? LEFT_JEWEL_ALT_DOWN_POSITON : RIGHT_JEWEL_ALT_DOWN_POSITON);
        if (sleep) this.opMode.sleep(50);
    }

    public void collectorUp() {
        servos.leftCollection.setPosition(COLLECTION_UP_POSITION);
        servos.rightCollection.setPosition(COLLECTION_UP_POSITION);
    }
    public void collectorDown() {
        servos.leftCollection.setPosition(COLLECTION_DOWN_POSITION);
        servos.rightCollection.setPosition(COLLECTION_DOWN_POSITION);
    }
    public void collectorOn() {
        motors.leftCollection.setPower(-1);
        motors.rightCollection.setPower(-1);
    }
    public void collectorOff() {
        motors.leftCollection.setPower(0);
        motors.rightCollection.setPower(0);

    }

    public void storeCryptoboxTarget() {
        cryptoboxTarget = imu.getAngularRotationX();
    }

    /**
     * This method expects that the robot is in the Cryptobox Safe Zone
     */
    public void alignWithCryptobox() {
        turningPIDController.turnToTarget(cryptoboxTarget, 1000);
        while (!alliance.detectTape(colorSensors.tape.red(), colorSensors.tape.green(), colorSensors.tape.blue())) {
            straightPIDController.move(0.2, 50);
        }
        while (!alliance.detectCryptoBoxDevider(colorSensors.cryptobox.red(), colorSensors.cryptobox.blue(), colorSensors.cryptobox.green())) {
            // FIXME: PulsarAuto does a bunch of other magic that we should really do too.
            straightPIDController.strafe(0.2 * alliance.getFlipFactor(), 50);
        }
    }

    /*
     * When this helper method is used in a turning function, correctionAngular
     * should be added to a power constant in order to make the wheels turn at a reasonable
     * speed. For small corrections during straight movement though, this is unnecessary.
     */
    @Override
    public void applyMotorPower(double velocityX, double velocityY,  double correctionAngular) {
        /*
         * Values are added and subtracted here based on the direction the wheels need to go on a
         * mecanum chassis to perform specific movements. For instance, Y velocity is always added,
         * because all motors go the same direction when moving forwards and backwards. Angular
         * movement and strafing require changes, because different wheels must move in different
         * directions to make the movement possible.
         */

        double frontLeftPower  = velocityY - velocityX + correctionAngular * CORRECTION_SCALAR;
        double frontRightPower = velocityY + velocityX - correctionAngular * CORRECTION_SCALAR;
        double backRightPower  = velocityY - velocityX - correctionAngular * CORRECTION_SCALAR;
        double backLeftPower   = velocityY + velocityX + correctionAngular * CORRECTION_SCALAR;

        motors.frontLeft.setPower(frontLeftPower);
        motors.frontRight.setPower(frontRightPower);
        motors.backLeft.setPower(backLeftPower);
        motors.backRight.setPower(backRightPower);
    }

    @Override
    public void stop() {
        motors.frontLeft.setPower(0);
        motors.frontRight.setPower(0);
        motors.backLeft.setPower(0);
        motors.backRight.setPower(0);
    }

    /**
     * Helper methods for compliance with RobotHardware
     */
    @Override
    public PIDCalculator.PIDTuning getTurningTuning() {
        return new PIDCalculator.PIDTuning(200, 0, 0);
    }

    @Override
    public PIDCalculator.PIDTuning getStraightTurning() {
        return new PIDCalculator.PIDTuning(100, 0, 0);
    }

    @Override
    public PIDCalculator.PIDTuning getStrafeTuning() {
        return this.getStraightTurning();
    }

    @Override
    public double getTurningAngleThreshold() {
        return 1;
    }

    @Override
    public Context getAppContext() {
        return appContext;
    }

    @Override
    public LinearOpMode getOpMode() {
        return opMode;
    }

    @Override
    public IMU getIMU() {
        return imu;
    }

    @Override
    public boolean getCanStrafe() {
        return true;
    }
}
