package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "BlueVisionCheesePlatterPark")
public class BlueVisionDuck extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "AUdem3D/////AAABmepzp1egIUCrnwRIbLuIVvIirZfrE6yO7yzl+4nX/cpFIvNXDph8rw66qfFZeX9QxP9gGNKTb26GubTBXLFPVrX/pKaNdcDrPCesrpE49iDgtcv4Rg59w0TwWVOrZo7h1WYKRw80AX0nutNBklVVGb2s7Liey79NqXcHY8a0Wwz5+2MjMb9XXoqPrIGJ2Wj+dDpvfItvhFpIVuaFkBR0VFGD9VuTJ7l2iBLa+57q6KDkC1qomNAM5UwN8u+HzA3RmxP+JCTaIIwQmO6IyeTxGiTcfHXMlExAQI9u7uHjFQtub4aOJ5v0I5MZDH1Q4U6mORm/9/pDNm5YnEY2UGqbEyOHdY5DGLpZmcRE/FsE+3bq";

    DcMotor fL = null;
    DcMotor fR = null;
    DcMotor bL = null;
    DcMotor bR = null;
    DcMotor iT = null;
    DcMotor dM = null;
    DcMotor iM = null;

    Servo a1 = null;
    Servo a2 = null;
    Servo aB = null;
    Servo i1 = null;
    Servo i2 = null;
    Servo hit = null;

    //    NormalizedColorSensor cS;
    TouchSensor tS;
    View relativeLayout;

    float[] hsvValues = new float[3];
    float gain = 4;
    boolean xButtonCurrentlyPressed;
    boolean xButtonPreviouslyPressed;

    AnalogInput potent = null;
    double number = .48;
    double position = .7;
    boolean armLoop = true;
    double runTimer = 0;
    String level = "";

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

    public class Arm extends Thread {
        private Thread t;
        private String threadName;

        Arm(String name) {
            threadName = name;
            System.out.println("Creating" + threadName);
        }

        public void moveArm(double arm1, double arm2) {
            a1.setPosition(arm1);
            a2.setPosition(arm2);

            armLoop = false;
        }
    }

    public class Rail extends Thread {
        private Thread t;
        private String threadName;

        Rail(String name) {
            threadName = name;
            System.out.println("Creating" + threadName);
        }

        public void moveRail(int position, double power) {
            iT.setTargetPosition(position);
            iT.setPower(.4);
        }
    }
        OpenCvWebcam webcam = null;

    public void init2(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new VisionCode.VisionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {

            }

        });


    }
    static class VisionPipeline extends OpenCvPipeline {

        public static Object position;

        public enum thingPos{
            ONE,
            TWO,
            THREE
        }

        Mat YCbCr = new Mat();
        Mat leftFrame;
        Mat centerFrame;
        Mat rightFrame;
        double leftAvgf;
        double centerAvgf;
        double rightAvgf;

        Mat output = new Mat();
        Scalar rectColor = new Scalar(255, 0,0);
        Scalar rect2Color = new Scalar(0,255,0);
        Scalar rect3Color = new Scalar(0,0,255);

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            Rect leftRect = new Rect(113, 200, 100, 100);
            Rect centerRect = new Rect(213, 200, 100, 100);
            Rect rightRect = new Rect(313, 200, 100, 100);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect,rectColor, 2);
            Imgproc.rectangle(output, centerRect,rect2Color, 2);
            Imgproc.rectangle(output, rightRect,rect3Color, 2);

            leftFrame = YCbCr.submat(leftRect);
            centerFrame = YCbCr.submat(centerRect);
            rightFrame = YCbCr.submat(rightRect);

            Core.extractChannel(leftFrame, leftFrame,0);
            Core.extractChannel(centerFrame, centerFrame,0);
            Core.extractChannel(rightFrame, rightFrame,0);

            Scalar leftAvg = Core.mean(leftFrame);
            Scalar centerAvg = Core.mean(centerFrame);
            Scalar rightAvg = Core.mean(rightFrame);

            leftAvgf = leftAvg.val[0];
            centerAvgf = centerAvg.val[0];
            rightAvgf = rightAvg.val[0];

            if(leftAvgf > centerAvgf && leftAvgf > rightAvgf){
                position = thingPos.ONE;
            }else if(centerAvgf > leftAvgf && centerAvgf > rightAvgf) {
                position = thingPos.TWO;
            }else if(rightAvgf > leftAvgf && rightAvgf > centerAvgf){
                position = thingPos.THREE;
            }

            return output;

        }

    }


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initualizing");

        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");
        iT = hardwareMap.dcMotor.get("iT");
//        iT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        iM = hardwareMap.dcMotor.get("iM");
        dM = hardwareMap.dcMotor.get("dM");

        a1 = hardwareMap.servo.get("a1");
        a2 = hardwareMap.servo.get("a2");
        aB = hardwareMap.servo.get("aB");
        i1 = hardwareMap.servo.get("i1");
        i2 = hardwareMap.servo.get("i2");
        hit = hardwareMap.servo.get("hit");

        tS = hardwareMap.touchSensor.get("tS");

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BlueVisionDuck.Arm initial = new BlueVisionDuck.Arm("First");
        initial.moveArm(.73, .27);

        iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iT.setTargetPosition(0);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setTargetPosition(0);
        iT.setPower(.1);

        aB.setPosition(1);

        boolean touch = tS.isPressed();

        waitForStart();

        //failsafe for missing block

        //Intake down
        hit.setPosition(1);
        Thread.sleep(500);

        telemetry.addData("Position " , VisionPipeline.position);
        telemetry.update();
        Thread.sleep(5000);

        Forward(700 , .3);
        Thread.sleep(500);

        //Face Warehouse
        TurnLeft(920 , .5);
        Thread.sleep(800);
        StrifeRight(1300 , .3);
        Thread.sleep(300);

        init2();

        //ONE == the left position
        if(VisionPipeline.position == VisionPipeline.thingPos.ONE)
        {
            //Different
            PlaceLow();
            Thread.sleep(500);
            Backward(150 , .3);
            Thread.sleep(1500);
            Drop();
            Thread.sleep(1300);
            Forward(180 , .3);
            Thread.sleep(500);
            Reset();
        }
        // TWO == center position
        else if(VisionPipeline.position == VisionPipeline.thingPos.TWO)
        {
            //Different
            PlaceMiddle();
            Backward(150 , .3);
            Thread.sleep(1500);
            Drop();
            Thread.sleep(1300);
            Forward(180 , .3);
            Thread.sleep(500);
            Reset();
        }
        //THREE == right position
        else if(VisionPipeline.position == VisionPipeline.thingPos.THREE)
        {
            //Different
            PlaceHigh();
            Backward(180 , .3);
            Thread.sleep(1500);
            Drop();
            Thread.sleep(1300);
            Forward(210 , .3);
            Thread.sleep(500);
            Reset();
        }

        //low
//        if (level == "low")
//        {
//            //Different
//            PlaceLow();
//            Thread.sleep(500);
//            Backward(150 , .3);
//            Thread.sleep(1500);
//            Drop();
//            Thread.sleep(1300);
//            Forward(180 , .3);
//            Thread.sleep(500);
//            Reset();
//        }

        //middle
//        if (level == "middle")
//        {
//            //Different
//            PlaceMiddle();
//            Backward(150 , .3);
//            Thread.sleep(1500);
//            Drop();
//            Thread.sleep(1300);
//            Forward(180 , .3);
//            Thread.sleep(500);
//            Reset();
//        }

        //high
//        if (level == "")
//        {
//            //Different
//            PlaceHigh();
//            Backward(180 , .3);
//            Thread.sleep(1500);
//            Drop();
//            Thread.sleep(1300);
//            Forward(210 , .3);
//            Thread.sleep(500);
//            Reset();
//        }

        StrifeLeft(2300 , .5);
        Thread.sleep(300);
        Forward(2000 , .5);
        Thread.sleep(300);



    }

    public void PlaceHigh() throws InterruptedException {
        aB.setPosition(.95);

        Thread.sleep(400);

        BlueVisionDuck.Arm movePlease = new BlueVisionDuck.Arm("First");
        movePlease.moveArm(.14 , .86);

        aB.setPosition(.65);

        Thread.sleep(600);

        aB.setPosition(.34);

        Thread.sleep(700);

        BlueVisionDuck.Rail moveR = new BlueVisionDuck.Rail("Second");
        moveR.moveRail(430 , .4);

//                iT.setTargetPosition(400);
//                iT.setPower(.4);

        telemetry.addData("iT position" , iT.getTargetPosition());
    }

    public void PlaceMiddle() throws InterruptedException {
        aB.setPosition(.9);

        Thread.sleep(100);

        BlueVisionDuck.Arm movePlease = new BlueVisionDuck.Arm("First");
        movePlease.moveArm(.32 , .68);

        aB.setPosition(.65);

        Thread.sleep(600);

        aB.setPosition(.52);

        Thread.sleep(700);

        BlueVisionDuck.Rail moveR = new BlueVisionDuck.Rail("Second");
        moveR.moveRail(400 , .4);
    }

    public void PlaceLow() throws InterruptedException {
        aB.setPosition(.9);

        Thread.sleep(100);

        BlueVisionDuck.Arm movePlease = new BlueVisionDuck.Arm("First");
        movePlease.moveArm(.4 , .6);

        aB.setPosition(.65);

        Thread.sleep(700);

        BlueVisionDuck.Rail moveR = new BlueVisionDuck.Rail("Second");
        moveR.moveRail(400 , .4);

        Thread.sleep(400);

        movePlease.moveArm(.5 , .5);
    }

    public void Reset() throws InterruptedException
    {
        BlueVisionDuck.Arm movePlease = new BlueVisionDuck.Arm("First");
        movePlease.moveArm(.4 , .6);

        Thread.sleep(400);

        telemetry.addData("Position" , iT.getCurrentPosition());

        iT.setTargetPosition(0);
        iT.setPower(.4);

        Thread.sleep(400);

        movePlease.moveArm(.73 , .27);

        aB.setPosition(1);
    }

    public void Drop()
    {
        aB.setPosition(.1);
    }

    public void ResetEncoders()
    {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ResetEncodersAll()
    {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunToPosition() {
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void RunToPositionAll() {
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void NoEncoders() {
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void On(double power)
    {
        fL.setPower(-power);
        fR.setPower(-power);
        bL.setPower(-power);
        bR.setPower(-power);
    }

    public void StrifeRight(int position , double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(-power);
        }
    }

    public void StrifeLeft(int position , double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }

    public void Backward(int position, double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
        }
    }

    public void TurnRight(int position, double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }

    public void TurnLeft(int position, double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(power);
            bR.setPower(-power);
        }
    }

    public void Forward(int position, double power) {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy()) {
            fL.setPower(-power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(-power);
        }
    }

    public void ScanForward(int position, double power) {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy()) {
            fL.setPower(-power);
            fR.setPower(-power * 1.4);
            bL.setPower(-power);
            bR.setPower(-power);
        }
    }

}




