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
            tm.addData("Correction: ", correctionAngular);
            tm.update();
            applyMotorPower(velocityXComponent, velocityYComponent, 0f, 0f, correctionAngular);
            long currSysTime = System.currentTimeMillis();
            elapsedTime = currSysTime - startTime;
            loopTime = currSysTime - loopTime;
        }
    }


        void applyMotorPower(double velocityX, double velocityY, double correctionX, double correctionY, double correctionAngular) {

            // Divide all corrections by 2000 to make sure we don't overflow. Not a good solution!!!

            double frontLeftPower = (velocityY + correctionY/2000)  - (velocityX + correctionX/2000) + correctionAngular/2000;
            double frontRightPower = (velocityY + correctionY/2000) - (velocityX + correctionX/2000) - correctionAngular/2000;
            double backLeftPower = (velocityY + correctionY/2000) + (velocityX + correctionX/2000) + correctionAngular/2000;
            double backRightPower = (velocityY + correctionY/2000) + (velocityX + correctionX/2000) - correctionAngular/2000;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

        void STOP() {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
}

