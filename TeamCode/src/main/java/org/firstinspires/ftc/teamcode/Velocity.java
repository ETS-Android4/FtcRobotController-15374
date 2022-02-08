package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous
public class Velocity extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "AUdem3D/////AAABmepzp1egIUCrnwRIbLuIVvIirZfrE6yO7yzl+4nX/cpFIvNXDph8rw66qfFZeX9QxP9gGNKTb26GubTBXLFPVrX/pKaNdcDrPCesrpE49iDgtcv4Rg59w0TwWVOrZo7h1WYKRw80AX0nutNBklVVGb2s7Liey79NqXcHY8a0Wwz5+2MjMb9XXoqPrIGJ2Wj+dDpvfItvhFpIVuaFkBR0VFGD9VuTJ7l2iBLa+57q6KDkC1qomNAM5UwN8u+HzA3RmxP+JCTaIIwQmO6IyeTxGiTcfHXMlExAQI9u7uHjFQtub4aOJ5v0I5MZDH1Q4U6mORm/9/pDNm5YnEY2UGqbEyOHdY5DGLpZmcRE/FsE+3bq";

    DcMotorEx fL = null;
    DcMotorEx fR = null;
    DcMotorEx bL = null;
    DcMotorEx bR = null;
    DcMotor iT = null;
    DcMotorEx dM = null;
    DcMotor iM = null;

    Servo a1 = null;
    Servo a2 = null;
    Servo aB = null;
    Servo i1 = null;
    Servo i2 = null;

    NormalizedColorSensor cS;
    View relativeLayout;

    float[] hsvValues = new float[3];
    float gain = 4;
    boolean xButtonCurrentlyPressed;
    boolean xButtonPreviouslyPressed;

    AnalogInput potent = null;
    double number = .48;
    double position = .7;
    boolean armLoop = true;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;
    private WebcamName webcamName = null;

    private boolean targetVisible = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initualizing");

        fL = hardwareMap.get(DcMotorEx.class , "fL");
        fR = hardwareMap.get(DcMotorEx.class , "fR");
        bL = hardwareMap.get(DcMotorEx.class , "bL");
        bR = hardwareMap.get(DcMotorEx.class , "bR");
        iT = hardwareMap.dcMotor.get("iT");
//        iT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        iM = hardwareMap.dcMotor.get("iM");
        dM = hardwareMap.get(DcMotorEx.class , "dM");

        a1 = hardwareMap.servo.get("a1");
        a2 = hardwareMap.servo.get("a2");
        aB = hardwareMap.servo.get("aB");
        i1 = hardwareMap.servo.get("i1");
        i2 = hardwareMap.servo.get("i2");

        cS = hardwareMap.get(NormalizedColorSensor.class, "cS");

        fR.setDirection(DcMotorEx.Direction.REVERSE);
        bR.setDirection(DcMotorEx.Direction.REVERSE);

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample(); // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }



        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        waitForStart();

        BackwardNE(1200);

//        Backward(700 , 600);
//        Thread.sleep(100);
//        TurnLeft(400 , 600);
//        Thread.sleep(100);
//        StrifeRight(1100 , 600);
//        Thread.sleep(100);
//        dM.setPower(-.8);
//        Thread.sleep(2000);



//        TurnLeft(750 , .3);
//        Thread.sleep(100);
//        Backward(800 , .3);
//        Thread.sleep(100);
//        StrifeRight(400 , .3);
//        Thread.sleep(100);
//        TurnRight(100 , .3);
//        Thread.sleep(100);
//        StrifeRight(200 , .3);
//        Thread.sleep(100);


//        TurnLeft(320 , .2);
//        Thread.sleep(100);
//        StrifeRight(930 , .3);
//        Thread.sleep(100);
//        Backward(100 , .2);
//        Thread.sleep(100);
//        StrifeRight(120 , .3);
//        Thread.sleep(100);
        //More
//        Backward(40 , .2);
//        Thread.sleep(100);


//        while (opModeIsActive()) {
//            // Explain basic gain information via telemetry
//            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
//            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");
//
//            // Update the gain value if either of the A or B gamepad buttons is being held
//            if (gamepad1.a) {
//                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
//                gain += 0.005;
//            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
//                gain -= 0.005;
//            }
//
//            // Show the gain value via telemetry
//            telemetry.addData("Gain", gain);
//
//            // Tell the sensor our desired gain value (normally you would do this during initialization,
//            // not during the loop)
//            cS.setGain(gain);
//
//            // Check the status of the X button on the gamepad
//            xButtonCurrentlyPressed = gamepad1.x;
//
//            // If the button state is different than what it was, then act
//            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
//                // If the button is (now) down, then toggle the light
//                if (xButtonCurrentlyPressed) {
//                    if (cS instanceof SwitchableLight) {
//                        SwitchableLight light = (SwitchableLight)cS;
//                        light.enableLight(!light.isLightOn());
//                    }
//                }
//            }
//            xButtonPreviouslyPressed = xButtonCurrentlyPressed;
//
//            // Get the normalized colors from the sensor
//            NormalizedRGBA colors = cS.getNormalizedColors();
//
//            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
//             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
//             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
//             * for an explanation of HSV color. */
//
//            // Update the hsvValues array by passing it to Color.colorToHSV()
//            Color.colorToHSV(colors.toColor(), hsvValues);
//
//            telemetry.addLine()
//                    .addData("Red", "%.3f", colors.red)
//                    .addData("Green", "%.3f", colors.green)
//                    .addData("Blue", "%.3f", colors.blue);
//            telemetry.addLine()
//                    .addData("Hue", "%.3f", hsvValues[0])
//                    .addData("Saturation", "%.3f", hsvValues[1])
//                    .addData("Value", "%.3f", hsvValues[2]);
//            telemetry.addData("Alpha", "%.3f", colors.alpha);
//            telemetry.update();
//
//            /* If this color sensor also has a distance sensor, display the measured distance.
//             * Note that the reported distance is only useful at very close range, and is impacted by
//             * ambient light and surface reflectivity. */
//            if (cS instanceof DistanceSensor) {
//                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) cS).getDistance(DistanceUnit.CM));
//            }
//
////            telemetry.update();
//
//            // Change the Robot Controller's background color to match the color detected by the color sensor.
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
//                }
//            });
        }

//        targets.activate();
//        while (!isStopRequested()) {
//
//            // check all the trackable targets to see which one (if any) is visible.
//            targetVisible = false;
//            for (VuforiaTrackable trackable : allTrackables) {
//                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                    targetVisible = true;
//
//                    // getUpdatedRobotLocation() will return null if no new information is available since
//                    // the last time that call was made, or if the trackable is not currently visible.
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        lastLocation = robotLocationTransform;
//                    }
//                    break;
//                }
//            }
//
//            // Provide feedback as to where the robot is located (if we know).
//            if (targetVisible) {
//                // express position (translation) of robot in inches.
//                VectorF translation = lastLocation.getTranslation();
//                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//            }
//            else {
//                telemetry.addData("Visible Target", "none");
//            }
//            telemetry.update();
//        }
//
//        // Disable Tracking when we are done;
//        targets.deactivate();




//    }




    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }


    protected void runSample() throws InterruptedException {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        hsvValues = new float[3];

        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        xButtonPreviouslyPressed = false;
        xButtonCurrentlyPressed = false;

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.


        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (cS instanceof SwitchableLight) {
            ((SwitchableLight) cS).enableLight(true);
        }
    }

    public void ResetEncoders()
    {
        fL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ResetEncodersAll()
    {
        fL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunToPosition() {
        fL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void RunToPositionAll() {
        fL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void NoEncoders() {
        fL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODERS);
        fR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODERS);
        bL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODERS);
        bR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void BackwardNE(int Veloc)
    {
        fL.setVelocity(Veloc);
        fR.setVelocity(Veloc);
        bL.setVelocity(Veloc);
        bR.setVelocity(Veloc);
    }

    public void StrifeRight(int position , int Veloc)
    {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setVelocity(-Veloc);
            fR.setVelocity(Veloc);
            bL.setVelocity(Veloc);
            bR.setVelocity(-Veloc);
        }
    }

    public void StrifeLeft(int position , int Veloc)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setVelocity(Veloc);
            fR.setVelocity(-Veloc);
            bL.setVelocity(-Veloc);
            bR.setVelocity(Veloc);
        }
    }

    public void Backward(int position, int Veloc)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setVelocity(Veloc);
            fR.setVelocity(Veloc);
            bL.setVelocity(Veloc);
            bR.setVelocity(Veloc);
        }
    }

    public void TurnRight(int position, int Veloc)
    {
        NoEncoders();

        ResetEncoders();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(position);

        RunToPosition();

        while (fL.isBusy())
        {
            fL.setVelocity(-Veloc);
            fR.setVelocity(Veloc);
            bL.setVelocity(-Veloc);
            bR.setVelocity(Veloc);
        }
    }

    public void TurnLeft(int position, int Veloc)
    {
        NoEncoders();

        ResetEncoders();


        fL.setTargetPosition(position);

        RunToPosition();

        while (fL.isBusy())
        {
            fL.setVelocity(Veloc);
            fR.setVelocity(-Veloc);
            bL.setVelocity(Veloc);
            bR.setVelocity(-Veloc);

            //Shows the current position of each motor
            telemetry.addData("fL - Position" , fL.getCurrentPosition());
            telemetry.addData("fR - Position" , fR.getCurrentPosition());
            telemetry.addData("bL - Position" , bL.getCurrentPosition());
            telemetry.addData("bR - Position" , bR.getCurrentPosition());
            telemetry.update();
        }
    }

    public void Forward(int position, int Veloc)
    {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setVelocity(-Veloc);
            fR.setVelocity(-Veloc);
            bL.setVelocity(-Veloc);
            bR.setVelocity(-Veloc);
        }
    }




}

