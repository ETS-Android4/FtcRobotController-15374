package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

@Autonomous
public class ForwardBackUp extends LinearOpMode {

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

//        cS = hardwareMap.get(NormalizedColorSensor.class, "cS");
//        tS = hardwareMap.digitalChannel.get("tS");
        tS = hardwareMap.touchSensor.get("tS");
//        tS = hardwareMap.get(DigitalChannel.class, "sensor_digital");
//        tS.setMode(DigitalChannel.Mode.INPUT);

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ForwardBackUp.Arm initial = new ForwardBackUp.Arm("First");
        initial.moveArm(.73, .27);

        aB.setPosition(1);

        boolean touch = tS.isPressed();

        waitForStart();

        //failsafe for missing block

        Forward(2200 , .3);
        Thread.sleep(100);




    }

    public void PlaceHigh() throws InterruptedException {
        aB.setPosition(.95);

        Thread.sleep(400);

        ForwardBackUp.Arm initial = new ForwardBackUp.Arm("First");
        initial.moveArm(.73, .27);

        aB.setPosition(.65);

        Thread.sleep(600);

        aB.setPosition(.34);

        Thread.sleep(600);

        iT.setTargetPosition(0);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setTargetPosition(400);
        iT.setPower(.4);
    }

    public void PlaceMiddle() throws InterruptedException {
        aB.setPosition(.9);

        Thread.sleep(100);

        ForwardBackUp.Arm initial = new ForwardBackUp.Arm("First");
        initial.moveArm(.73, .27);

        aB.setPosition(.65);

        Thread.sleep(600);

        aB.setPosition(.52);

        Thread.sleep(600);

        iT.setTargetPosition(0);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setTargetPosition(400);
        iT.setPower(.4);
    }

    public void PlaceLow() throws InterruptedException {
        aB.setPosition(.9);

        Thread.sleep(100);

        ForwardBackUp.Arm initial = new ForwardBackUp.Arm("First");
        initial.moveArm(.73, .27);

        aB.setPosition(.65);

        Thread.sleep(600);

        iT.setTargetPosition(0);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setTargetPosition(400);
        iT.setPower(.4);
    }

    public void Reset() throws InterruptedException {
        iT.setTargetPosition(0);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setTargetPosition(-400);
        iT.setPower(.4);

        Thread.sleep(1000);

        ForwardBackUp.Arm initial = new ForwardBackUp.Arm("First");
        initial.moveArm(.73, .27);

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
            fR.setPower(-power);
            bL.setPower(-power + .2);
            bR.setPower(-power);
        }
    }

}




