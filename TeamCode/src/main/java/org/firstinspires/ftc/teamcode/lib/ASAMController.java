package org.firstinspires.ftc.teamcode.lib;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

/**
 * Created by ariporad on 2017-12-09.
 */

public class ASAMController {
        public static float MOTOR_SPEED_CONVERSION_FACTOR = 1f;

        public static double interpolateTweenCurve(long totalRunTime, long elapsedTime, float startSpeed, float endSpeed, long accelTimeMillis, Telemetry t) {
            double ret;
            String curveSection;
            if (elapsedTime <= accelTimeMillis) {
                curveSection = "Accel";
                ret = ((startSpeed - endSpeed) / 2) * Math.cos((elapsedTime * Math.PI) / accelTimeMillis) + (startSpeed + endSpeed) / 2;
            } else if (elapsedTime < (totalRunTime - accelTimeMillis)) {
                curveSection = "FullS";
                ret = endSpeed;
            } else {
                curveSection = "Decel";
                ret = ((endSpeed - startSpeed) / 2) * Math.cos(((Math.PI * (totalRunTime - accelTimeMillis - elapsedTime))/accelTimeMillis)) + (startSpeed + endSpeed) / 2;
                //return ((endSpeed - startSpeed) / 2) * Math.cos(((Math.PI * (accelTimeMillis - (totalRunTime - elapsedTime)))/accelTimeMillis)) + (startSpeed + endSpeed) / 2;
            }
            Log.d("Redshift/ASAM",
                    "Mode: " + String.valueOf(curveSection) +
                    ", Speed: " + new DecimalFormat("#.#####").format(ret) +
                    ".\t\t\ttotalRunTime: "+String.valueOf(totalRunTime)+
                    ", elapsedTime: "+String.valueOf(elapsedTime) +
                    ", startSpeed: "+ String.valueOf(startSpeed) +
                    ", endSpeed: " + String.valueOf(endSpeed) +
                    ", accelTimeMillis: " + String.valueOf(accelTimeMillis)
            );
            return ret;
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
