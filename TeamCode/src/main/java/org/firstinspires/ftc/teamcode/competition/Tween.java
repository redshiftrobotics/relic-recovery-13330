package org.firstinspires.ftc.teamcode.competition;

/**
 * Created by adam on 12/7/17.
 */
public class Tween {
    public static float MOTOR_SPEED_CONVERSION_FACTOR = 1f;

    public static double interpolateTweenCurve(long totalRunTime, long elapsedTime, float startSpeed, float endSpeed, long accelTimeMillis) {
        if (elapsedTime <= accelTimeMillis) {
            return ((startSpeed - endSpeed) / 2) * Math.cos((elapsedTime * Math.PI) / accelTimeMillis) + (startSpeed + endSpeed) / 2;
        } else if (elapsedTime < totalRunTime - accelTimeMillis) {
            return endSpeed;
        } else {
            return ((endSpeed - startSpeed) / 2) * Math.cos((Math.PI * (totalRunTime - accelTimeMillis - elapsedTime)) / accelTimeMillis) + (startSpeed + endSpeed) / 2;
        }
    }

    public static double totalMovementTimeWithTweenCurve(int encoderTicks, float accelTime) {
         return 2 * accelIntegral(accelTime, 0, accelTime) * MOTOR_SPEED_CONVERSION_FACTOR;
    }

    public static double accelIntegral(float accelTime, float lowerBound, float upperBound) {
        return accelAntiderivative(accelTime, upperBound) - accelAntiderivative(accelTime, lowerBound);
    }

    public static double accelAntiderivative(float accelTime, float t) {
        return (accelTime * Math.sin((Math.PI * t) / accelTime)) / Math.PI;
    }
}
