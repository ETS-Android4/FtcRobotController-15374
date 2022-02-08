package org.firstinspires.ftc.teamcode;

import android.net.NetworkInfo;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp (name = "Steve")
public class Steve extends LinearOpMode {
    //private static final Object WORK = new Thread("Work");

    //    public static void main (String arg[])
//    {
//
//    }
    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;
    public DcMotor iT = null;
    public DcMotor dM = null;
    public DcMotor iM = null;

    public TouchSensor limitM = null;
    public TouchSensor limitT = null;

    public Servo a1 = null;
    public Servo a2 = null;
    public Servo aB = null;
    public Servo i1 = null;
    public Servo i2 = null;
    public Servo hit = null;

    public AnalogInput potent = null;
    double number = .48;
    double position = .7;
    boolean armLoop = true;
    //.24 level

    private final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver LED;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    public enum DisplayKind {
        MANUAL,
        AUTO
    }

    //@Override
    public void init2()
    {
        displayKind = SampleRevBlinkinLedDriver.DisplayKind.AUTO;

        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        pattern = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
        LED.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    //@Override
    public void loop2()
    {
        handleGamepad();

        if (displayKind == SampleRevBlinkinLedDriver.DisplayKind.AUTO) {
            doAutoDisplay();
        } else {
            /*
             * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.
             */
        }
    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
     */
    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad2.a) {
            setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (gamepad2.b) {
            setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.left_bumper)) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        } else if ((displayKind == SampleRevBlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.right_bumper)) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        }
    }

    protected void setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        LED.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }

    public class Arm extends Thread{
        private Thread t;
        private String threadName;

        Arm (String name){
            threadName = name;
            System.out.println("Creating" + threadName);
        }

        public void moveArm (double arm1 , double arm2){
            a1.setPosition(arm1);
            a2.setPosition(arm2);

            armLoop = false;
        }

//        public void moveRail (int position)
//        {
//            .setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            .setTargetPosition(position);
//        }
    }

    public void runOpMode() throws InterruptedException{

        telemetry.addData("Status", "Initializing");

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

//        limitM = hardwareMap.touchSensor.get("limitM");
//        limitT = hardwareMap.touchSensor.get("limitT");

        //Either or way
        potent = hardwareMap.analogInput.get("potent");
//
//        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //iT.setMode(DcMotor.RunMode.RESET_ENCODERS);

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        iT.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status" , "Initializd");

        Arm initial = new Arm("First");
        initial.moveArm(.73 , .27);

        aB.setPosition(1);

        hit.setPosition(1);

        init2();
        loop2();

        setDisplayKind(SampleRevBlinkinLedDriver.DisplayKind.MANUAL);

        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("Status", "Runnging");

            telemetry.addLine("potent Voltage" + potent.getVoltage());
            telemetry.update();

            //Gamepad1

            //Driving Controls

            //Forward
            fL.setPower(gamepad1.left_stick_y);
            bL.setPower(gamepad1.left_stick_y);
            fR.setPower(gamepad1.left_stick_y);
            bR.setPower(gamepad1.left_stick_y);

            //Rotating
            fL.setPower(-gamepad1.left_stick_x);
            bL.setPower(-gamepad1.left_stick_x);
            fR.setPower(gamepad1.left_stick_x);
            bR.setPower(gamepad1.left_stick_x);

            //Other Controls

            //Intake
            iM.setPower(-gamepad1.right_stick_y);

            //Strifing left, moving side to side using mecanum wheels
            while (gamepad1.right_bumper) {
                fL.setPower(-1);
                bL.setPower(1);
                fR.setPower(1);
                bR.setPower(-1);
            }

            //Strifing right, moving side to side using mecanum wheels
            while (gamepad1.left_bumper) {
                fL.setPower(1);
                bL.setPower(-1);
                fR.setPower(-1);
                bR.setPower(1);
            }

            //Reset button
            if (gamepad1.y)
            {
                iT.setTargetPosition(0);
                iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                iT.setTargetPosition(-400);
                iT.setPower(.4);

                Thread.sleep(1000);

                Arm movePlease = new Arm("First");
                movePlease.moveArm(.73 , .27);

                aB.setPosition(1);
            }

            //Top level
            while (gamepad1.x)
            {
                aB.setPosition(.95);

                Thread.sleep(400);

                Arm movePlease = new Arm("First");
                movePlease.moveArm(.14 , .86);

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

                telemetry.addData("iT position" , iT.getTargetPosition());
            }

            //Middle Level
            while (gamepad1.b)
            {
                aB.setPosition(.9);

                Thread.sleep(100);

                Arm movePlease = new Arm("First");
                movePlease.moveArm(.32 , .68);

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

                telemetry.addData("iT position" , iT.getTargetPosition());
            }

            //Lower Level
            while (gamepad1.a)
            {
                aB.setPosition(.9);

                Thread.sleep(100);

                Arm movePlease = new Arm("First");
                movePlease.moveArm(.4 , .6);

                aB.setPosition(.65);

                Thread.sleep(600);

                iT.setTargetPosition(0);
                iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                iT.setTargetPosition(400);
                iT.setPower(.4);

                telemetry.addData("iT position" , iT.getTargetPosition());
            }

            //Drop button
            if (gamepad1.right_trigger > .3)
            {
                aB.setPosition(.1);
            }

            //Duck Spinner
            if (gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                dM.setPower(-.8);
            }
            else if (!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                dM.setPower(0);
            }

            if (gamepad1.dpad_down &&!gamepad1.dpad_up)
            {
                dM.setPower(.8);
            }
            else if (!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                dM.setPower(0);
            }

            //Gamepad2

            //Lights

            if (gamepad2.a)
            {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                LED.setPattern(pattern);

                init2();

                display = telemetry.addData("Display Kind: ", displayKind.toString());
                patternName = telemetry.addData("Pattern: ", pattern.toString());
            }


        }


    }



}
