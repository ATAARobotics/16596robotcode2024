package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.ReverseTeamElementPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private boolean isRed;

    public void setRed(boolean red) {
        isRed = red;
    }

    public enum Position{
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    };
    ReverseTeamElementPipeline pipeline;
    OpenCvWebcam cam;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        cam.setMillisecondsPermissionTimeout(5000);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
    public void init() {
        pipeline = new ReverseTeamElementPipeline(isRed);
        cam.setPipeline(pipeline);
    }

    public void start() {
        cam.stopStreaming();
    }

    public Position detectElement() {
        Position target = Position.RIGHT;
        if (pipeline.result != ReverseTeamElementPipeline.Result.Unknown) {
            if (pipeline.result == ReverseTeamElementPipeline.Result.Left) {
                target = Position.LEFT;
            } else if (pipeline.result == ReverseTeamElementPipeline.Result.Middle) {
                target = Position.MIDDLE;
            }

//            cam.stopStreaming();
        }
        return target;
    }

    public void printTelemetry() {
        telemetry.addData("Frame Count", cam.getFrameCount());
        telemetry.addData("FPS", cam.getFps());
        telemetry.addData("Total Frame Time ms", cam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", cam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", cam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", cam.getCurrentPipelineMaxFps());
    }

}
