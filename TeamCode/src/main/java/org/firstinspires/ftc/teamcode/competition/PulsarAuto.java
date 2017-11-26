package org.firstinspires.ftc.teamcode.competition;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.ColumnController;
import org.redshiftrobotics.lib.config.ConfigurationManager;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;


abstract public class PulsarAuto extends LinearOpMode {

    private ConfigurationManager allianceConf;

    abstract protected Alliance getAlliance();
    abstract protected StartPosition getStartPosition();

    protected enum Alliance {
        BLUE, RED;

        public TargetJewelPosition getTargetJewel(StartPosition pos, ColorSensor colorSensor) {
            int red = colorSensor.red();
            int blue = colorSensor.blue();

            Alliance frontJewel;

            if (red > 3 && red > blue) frontJewel = RED;
            else if (blue > 3 && blue > red) frontJewel = BLUE;
            else return TargetJewelPosition.NONE;

            if (frontJewel == this) return TargetJewelPosition.BACK;
            else return TargetJewelPosition.FRONT;
        }

        public ConfigurationManager getConf() {
            return ConfigurationManager.getSharedInstance().getConfig("auto").getConfig(this == BLUE ? "blue" : "red");
        }

        public double getAngleToAlignWithCryptobox(StartPosition pos) {
            if (pos == StartPosition.FRONT) {
                if (this == BLUE) return -90;
                return 90;
            } else {
                return 0;
            }
        }

        public double getDistanceToAlignWithColumn(StartPosition pos, RelicRecoveryVuMark column) {
            if (this == BLUE) {
                if (pos == StartPosition.FRONT) {
                    switch (column) {
                        case LEFT: return 10;
                        case CENTER: return 30;
                        case RIGHT: return 50;
                    }
                } else {
                    switch (column) {
                        case LEFT: return 10;
                        case CENTER: return 30;
                        case RIGHT: return 50;
                    }
                }
            } else {
                if (pos == StartPosition.FRONT) {
                    switch (column) {
                        case RIGHT: return 10;
                        case CENTER: return 30;
                        case LEFT: return 50;
                    }
                } else {
                    switch (column) {
                        case RIGHT: return 10;
                        case CENTER: return 30;
                        case LEFT: return 50;
                    }
                }
            }
            return 0;
        }
    }
    protected enum StartPosition { BACK, FRONT }

    protected enum TargetJewelPosition { FRONT, BACK, NONE }

    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;

    protected Servo jewel;
    protected Servo leftJewel;
    protected Servo rightJewel;

    protected ColorSensor jewelDetector;
    protected ColorSensor leftJewelDetector;
    protected ColorSensor rightJewelDetector;

    protected DcMotor conveyor;

    protected StartPosition startPosition = getStartPosition();
    protected Alliance alliance = getAlliance();
    protected ConfigurationManager conf = alliance.getConf();

    protected boolean PULSAR_SIMPLE_AUTO = false;

     MecanumRobot robot;

     ColumnController columnController;

     BNO055IMU imu;

     void moveStraight(float speed, long timeout, double cmDistance) {
        robot.moveStraight(speed, timeout, cmDistance);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            ConfigurationManager.setup("default");
        } catch (Exception e) {
            telemetry.addLine("ERROR!!!!! EXITING");
            telemetry.update();
            Log.e("PulsarAuto", "configuration manager error", e);
            return;
        }

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        columnController = new ColumnController(hardwareMap);
        leftJewel = hardwareMap.servo.get("leftJewel");
        rightJewel = hardwareMap.servo.get("rightJewel");
        leftJewelDetector = hardwareMap.colorSensor.get("leftJewelDetector");
        rightJewelDetector = hardwareMap.colorSensor.get("rightJewelDetector");
        conveyor = hardwareMap.dcMotor.get("conveyor");

        leftJewel.setPosition(Alliance.BLUE.getConf().getDouble("jewelUpPosition"));
        rightJewel.setPosition(Alliance.RED.getConf().getDouble("jewelUpPosition"));

        jewel = (alliance == Alliance.BLUE) ? leftJewel : rightJewel;
        jewelDetector = (alliance == Alliance.BLUE) ? leftJewelDetector : rightJewelDetector;

        robot = new MecanumRobot(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                imu,
                this,
                telemetry
        );
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();


        jewel.setPosition(conf.getDouble("jewelDownPosition"));

        telemetry.addLine("Jewel Lowered");
        telemetry.update();

        Thread.sleep(1000);

        TargetJewelPosition targetJewelPosition = alliance.getTargetJewel(startPosition, jewelDetector);

        telemetry.addData("saw jewel", targetJewelPosition.toString());
        if (targetJewelPosition == TargetJewelPosition.NONE) {
            telemetry.update();
            telemetry.addLine("Got None, moving");
            jewel.setPosition(conf.getDouble("jewelDownAltPosition"));

            telemetry.addLine("Jewel Lowered");
            telemetry.update();

            Thread.sleep(1000);

            targetJewelPosition = alliance.getTargetJewel(startPosition, jewelDetector);
        }
        telemetry.update();

        Thread.sleep(1000);

        switch (targetJewelPosition) {
            case FRONT: break; // We do this later
            case BACK:
                double jewelKnockoffAngle = conf.getDouble("frontJewelKnockOffAngle");
                robot.turn(jewelKnockoffAngle, 2000);
                jewel.setPosition(conf.getDouble("jewelUpPosition"));
                robot.turn(-jewelKnockoffAngle, 2000);
                break;
            case NONE:
                jewel.setPosition(conf.getDouble("jewelUpPosition"));
                Thread.sleep(1000);
                break;
        }

        moveStraight(1, 5000, conf.getParent().getDouble("distanceToClearStone"));
        Thread.sleep(1000);

        jewel.setPosition(allianceConf.getDouble("jewelUpPosition"));
        Thread.sleep(1000);

        if (PULSAR_SIMPLE_AUTO) {
            telemetry.addLine("Simple auto only, exiting.");
            telemetry.update();
            return;
        }

        telemetry.addData("angle", alliance.getAngleToAlignWithCryptobox(startPosition));
        telemetry.update();

        robot.turn(alliance.getAngleToAlignWithCryptobox(startPosition), 2000);
        robot.STOP();
        Thread.sleep(10000);

        moveStraight(1, 5000, alliance.getDistanceToAlignWithColumn(startPosition, columnController.getColumn()));
        robot.STOP();
        Thread.sleep(1000);

        robot.turn(conf.getDouble("angleToFaceCryptobox"), 2000);
        robot.STOP();
        Thread.sleep(1000);

        conveyor.setPower(1.0);
        robot.STOP();
        Thread.sleep(1000);
    }
}
