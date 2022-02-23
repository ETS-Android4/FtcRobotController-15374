package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

class VisionCode extends OpMode {

        OpenCvWebcam webcam = null;

    @Override
    public void init(){

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new VisionPipeline());

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


    @Override
    public void loop(){

    }
    static class VisionPipeline extends OpenCvPipeline {

        Mat YCbCr = new Mat();
        Mat leftFrame;
        Mat centerFrame;
        Mat rightFrame;
        double leftAvgf;
        double centerAvgf;
        double rightAvgf;
        int position = 0;

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
                position = 1;
            }else if(centerAvgf > leftAvgf && centerAvgf > rightAvgf) {
                position = 2;
            }else if(rightAvgf > leftAvgf && rightAvgf > centerAvgf){
                position = 3;
            }

            return output;


        }

        }

    }


