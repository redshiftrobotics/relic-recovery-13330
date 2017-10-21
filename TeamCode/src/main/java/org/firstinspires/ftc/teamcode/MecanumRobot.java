package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.redshiftrobotics.lib.*;

/**
 * Created by adam on 10/11/17.
 */
public class MecanumRobot {
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    CoordinatePIDController xyController;
    IMUPIDController imupidController;
    IMUImpl imuImpl;
    LinearOpMode context;
    Telemetry tm;

    static float ANGLE_THRESHOLD = 0.1f;

    static float P_TUNING = 150f, I_TUNING = 2.0e-4f, D_TUNING = 0f;
    static boolean DEBUG = true;

    public MecanumRobot(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, BNO055IMU imu, DistanceDetector detector, LinearOpMode context, Telemetry tm) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;

        imuImpl = new IMUImpl(imu);
        xyController = new CoordinatePIDController(detector);
        imupidController = new IMUPIDController(imuImpl);
        this.context = context;
        this.tm = tm;
    }

    // THIS WILL PROBABLY NOT BE USED THIS SEASON. We don't really need coordinate PID, since
    // the encoder idea is scrapped.
    public void MoveTo(float x, float y, float targetAngle, float speed, float xTolerance, float yTolerance, float timeout) {

        // Check for speed overflow. 
        if (speed > 1) speed = 1;
        if (speed < -1) speed = -1;

        xyController.setXYTarget(x, y);
        imupidController.setTarget(targetAngle);

        Vector2D movement = new Vector2D(x, y);

        double angle = movement.GetDirection();

        Vector2D velocity = new Vector2D(0, 0);
        velocity.SetPolar(speed, angle);

        double velocityXComponent = velocity.GetXComponent();
        double velocityYComponent = velocity.GetYComponent();

        long elapsedTime = 0;
        long startTime = System.currentTimeMillis();
        long loopTime = System.currentTimeMillis();


        while ((Math.abs(xyController.Px) >= xTolerance
                || Math.abs(xyController.Px) >= yTolerance) && elapsedTime <= timeout) {

            double[] correctionXY = xyController.calculatePID(loopTime/1000);
            double correctionAngular = imupidController.calculatePID(loopTime/1000);
            applyMotorPower(velocityXComponent, velocityYComponent, correctionXY[0], correctionXY[1], correctionAngular);

            long currSysTime = System.currentTimeMillis();
            elapsedTime = currSysTime - startTime;
            loopTime = currSysTime - loopTime;
        }
    }

    public void MoveStraight(float speed, double angle, long timeout) {
        imupidController.clearData();
        imupidController.setTuning(P_TUNING, I_TUNING, D_TUNING);
        if (DEBUG) {
            tm.addData("P: " + imupidController.P + " I: " + imupidController.I + " D: " + imupidController.D, "");
            tm.update();
        }

        // Check for speed overflow.
        if (speed > 1) speed = 1;
        if (speed < -1) speed = -1;


        Vector2D velocity = new Vector2D(0, 0);
        velocity.SetPolar(speed, angle);

        double velocityXComponent = velocity.GetXComponent();
        double velocityYComponent = velocity.GetYComponent();

        long elapsedTime = 0;
        long startTime = System.currentTimeMillis();
        long loopTime = System.currentTimeMillis();



        while (elapsedTime <= timeout && context.opModeIsActive()) {
            double correctionAngular = imupidController.calculatePID(loopTime/1000);

            if (DEBUG) {
                tm.addData("P: " + imupidController.P + "I: " + imupidController.I + "D: " + imupidController.D, "");
                tm.update();
            }
            applyMotorPower(velocityXComponent, velocityYComponent, 0f, 0f, correctionAngular);
            long currSysTime = System.currentTimeMillis();
            elapsedTime = currSysTime - startTime;
            loopTime = currSysTime - loopTime;
        }
    }

    public void Turn(float robotAngle, long timeout) {
        // Set tuning for turning
        imupidController.setTuning(P_TUNING, I_TUNING, D_TUNING);

        // Clear out past data.
        imupidController.clearData();



        if (DEBUG) {
            tm.addData("Target: ", imupidController.target);
            tm.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                tm.addData("Error!", "");
                tm.update();
            }
            imupidController.addTarget(robotAngle);
            tm.addData("New Target: ", imupidController.target);
            tm.update();
        }


        long elapsedTime = 0;
        long startTime = System.currentTimeMillis();
        long loopTime = System.currentTimeMillis();

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        while (elapsedTime <= timeout && context.opModeIsActive()) {
            double correctionAngular = imupidController.calculatePID(loopTime/1000);
            tm.addData("P: ", imupidController.P);
            tm.update();
            // Apply motor
            applyMotorPower(0, 0, 0, 0,  0.5+correctionAngular);
            long currSysTime = System.currentTimeMillis();
            elapsedTime = currSysTime - startTime;
            loopTime = currSysTime - loopTime;

            // Our tolerance condition.
            if (withinThreshold(imupidController.P, ANGLE_THRESHOLD) && withinThreshold(imupidController.lastError, ANGLE_THRESHOLD)) {
                break;
            }
        }
    }


        void applyMotorPower(double velocityX, double velocityY, double correctionX, double correctionY, double correctionAngular) {

            // Divide all corrections by 2000 to make sure we don't overflow. Not a good solution!!!

            double frontLeftPower = velocityY  - velocityX  - correctionAngular/2000;
            double frontRightPower = velocityY + velocityX  + correctionAngular/2000;
            double backRightPower = velocityY - velocityX + correctionAngular/2000;
            double backLeftPower = velocityY + velocityX - correctionAngular/2000;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

        public boolean withinThreshold(float value, float tolerance) {
            return Math.abs(value) <= tolerance;
        }

        public void STOP() {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
}

