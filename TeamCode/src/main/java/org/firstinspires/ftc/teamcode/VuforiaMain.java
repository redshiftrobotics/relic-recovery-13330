package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by adam on 9/12/17.
 */
public class VuforiaMain {
    private final String VUFORIA_KEY = "AZSn6x3/////AAAAGQUiAVV7BUM5p1/oUpgt2zd2gpH6mH3RDbbzWwc6oPE80fZ61JSft68k7bnar35QeFYAffqqC4lASNO+ufDo3YkAAmrqm7xttuFSQCwStUUwxj6smqRehkzjIG9Ud/qMUKwtZ477dal9IayK0S/meM6t8xQpLOfGpFesBjXBxqaO092Uz3ab+O+Y3px+tSwo+w7NTqDKy6QhJnju6vyqLN10tXhzAYCdsl0tPmNoYfieelsQNAfQrTO0onkzGrvJXsSF+J+eVbwVUtdn1+SK2MWyVQHks/aXvin929RYaMTgxiAz6GwmKOHR5/S4XarDBz48mKGSnxB00OOg8QxFSWkKPsHen5b9ZQpVFwcqdzz0";

    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    public VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    public Orientation angle = new Orientation();
    public Position position = new Position();

    //List of all trackable objects
    //TODO: Add to this list
    List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();

    // The types of images we can potentially recognize
    public enum ImageType {
       LEFT, CENTER, RIGHT
    }

    public void UpdateLocation() {
        for (VuforiaTrackable tracker : trackables) {
            OpenGLMatrix latestLocation = ((VuforiaTrackableDefaultListener) tracker).getUpdatedRobotLocation();

            if (latestLocation != null) {
                lastKnownLocation = latestLocation;
            }
        }

        angle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        position.x = lastKnownLocation.getTranslation().get(0);
        position.y = lastKnownLocation.getTranslation().get(1);
        position.z = lastKnownLocation.getTranslation().get(2);
    }


    public Position getPosition() {
        return this.position;
    }

    public Orientation getOrientation() {
        return this.angle;
    }


    public void Setup(ImageType image, VuforiaLocalizer.CameraDirection direction){
       /* VuforiaLocalizer.Parameters parameters;
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = direction; //sets the camera used by vuforia
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        //sets up the target to the correct image based off the input
        switch (image) {
            case Wheels:
                target = visionTargets.get(0);
                target.setName("Wheels Target");
                target.setLocation(createMatrix(0, 0, 0, 90, 0, 90));
                break;
            case Tools:
                target = visionTargets.get(1);
                target.setName("Tools Target");
                target.setLocation(createMatrix(0, 0, 0, 90, 0, 90));
                break;
            case Legos:
                target = visionTargets.get(2);
                target.setName("Legos Target");
                target.setLocation(createMatrix(0, 0, 0, 90, 0, 90));
                break;
            case Gears:
                target = visionTargets.get(3);
                target.setName("Gears Target");
                target.setLocation(createMatrix(0, 0, 0, 90, 0, 90));
                break;
            default:
                return;
        }

        phoneLocation = createMatrix(0, 0, 0, 0, 0, 0);

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        visionTargets.activate();*/
        //TODO: setup vuforia here; should initialize location of phone and image targets.
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    private String formatMatix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}

