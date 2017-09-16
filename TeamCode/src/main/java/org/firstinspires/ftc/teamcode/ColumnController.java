package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This class is designed to keep track of which column gives bonus points.
 * It is not designed to track the vuforia targets, it only detects them once.
 * @author Duncan McKee
 * @version 1.0.0
 */
public class ColumnController {
    private VuforiaLocalizer vuforia;
    public RelicRecoveryVuMark vuMark;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;

    public ColumnController(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //This is our key for vuforia
        parameters.vuforiaLicenseKey = "AZSn6x3/////AAAAGQUiAVV7BUM5p1/oUpgt2zd2gpH6mH3RDbbzWwc6oPE80fZ61JSft68k7bnar35QeFYAffqqC4lASNO+ufDo3YkAAmrqm7xttuFSQCwStUUwxj6smqRehkzjIG9Ud/qMUKwtZ477dal9IayK0S/meM6t8xQpLOfGpFesBjXBxqaO092Uz3ab+O+Y3px+tSwo+w7NTqDKy6QhJnju6vyqLN10tXhzAYCdsl0tPmNoYfieelsQNAfQrTO0onkzGrvJXsSF+J+eVbwVUtdn1+SK2MWyVQHks/aXvin929RYaMTgxiAz6GwmKOHR5/S4XarDBz48mKGSnxB00OOg8QxFSWkKPsHen5b9ZQpVFwcqdzz0";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        storeColumn();
    }

    /**
     * This function is designed only to run once during the init phase.
     * However, it may be useful to have it run later if there is a problem so it is kept as public.
     */
    public void storeColumn(){
        relicTrackables.activate();
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.deactivate();
    }

    /**
     * A way to keep track of our movements for autonomous for placing boxes in the correct box for bonus points.
     * @return An int between -1 and 1 to multiply a movement parallel to the Cryptobox.
     * @throws Exception
     */
    public int movementMultiplier() throws Exception {
        switch (vuMark){
            case LEFT:
                return -1;
            case CENTER:
                return 0;
            case RIGHT:
                return 1;
            default:
                throw new Exception("Did not have a target");
        }
    }
}
