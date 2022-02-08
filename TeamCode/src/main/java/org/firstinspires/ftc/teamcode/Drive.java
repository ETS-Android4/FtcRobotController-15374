package org.firstinspires.ftc.teamcode;

import android.net.NetworkInfo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.StateMachine;

@Disabled
@TeleOp (name = "Drive")
public class Drive extends LinearOpMode {
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

    public AnalogInput potent = null;
    double number = .48;
    double position = .7;
    boolean armLoop = true;
    //.24 level

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

//    Arm = new Arm();

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

        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("Status", "Runnging");

//            if (gamepad2.a)
//            {
//                aB.setPosition(1);
//            }

            telemetry.addLine("potent Voltage" + potent.getVoltage());
            telemetry.update();

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

            iM.setPower(-gamepad1.right_stick_y);

            //Change while if issue

            //End up with velocity instead of whatever
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

            while (gamepad2.dpad_up){
                a1.setPosition(1);
                a2.setPosition(0);
            }

            while (gamepad2.dpad_down){
                a1.setPosition(0);
                a2.setPosition(1);
            }

            if (gamepad2.a)
            {
                aB.setPosition(0);
            }

            if (gamepad2.b)
            {
                aB.setPosition(1);
            }

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
            //Set
            while (gamepad1.x)
            {
                aB.setPosition(.95);

                Thread.sleep(400);

                Arm movePlease = new Arm("First");
                movePlease.moveArm(.14 , .86);

                aB.setPosition(.65);

                Thread.sleep(600);

                aB.setPosition(.4);

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
            //
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
            //
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

            if (gamepad1.right_trigger > .3)
            {
                aB.setPosition(.1);
            }

            if (gamepad1.dpad_right)
            {
                i1.setPosition(.6);
                i2.setPosition(.4);
            }

            if (gamepad1.dpad_left)
            {
                i1.setPosition(.3);
                i2.setPosition(.7);
            }

            //Put on joystick
            //Why no work?

            //D-pad down faster regardless of direction
            if (gamepad1.dpad_up)
            {
                dM.setPower(-1);
            }
            else
            {
                dM.setPower(0);
            }

            if (gamepad1.dpad_down)
            {
                iT.setTargetPosition(1);
            }


        }


    }



}
