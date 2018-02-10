package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Disabled
@Autonomous(name="Test", group="Autonomous")
public class Test extends LinearOpMode {

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AbAk0Qv/////AAAAGReJq1q3oEBBpL6gFlJ4tEtmB5O0nOAttAl5uqbuAxyfQdmBpSKniaAcsJo+qSLLEp/U9eOlrSyEj0vPBZz+fw9oatXAs9HH7Bvytz8M5NBokl/VZPfHhRKwVxc/E/SyxvwenQh/NrpWqHH1Jia2BLXEnqy0I4ANVepkbL4wZSYj+zhFrHx0UldPm+6XLJkKeZnf21674Wc00WeV9oVn9rpC8FTy6XOEgcgW01iYQPBJN6gLcyvTE5965w/rETGbzU8yPrdmcl1eCZBDH7vBDI1qBwOFCJSEKzBQ7Lc20gSZvvCTF/1mZN/jRXirc+t9KLGU+1k42yrfXL6fTWz3QZHt3eBSpwwN1mCJaFqi5soE";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        //Wait for the play button to be hit
        waitForStart();
        //Loop until we know what we are looking at
        //AKA nothing
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        telemetry.addData("VuMark", "%s visible", vuMark);

    }

    private String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
