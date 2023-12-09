package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.ReverseTeamElementPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    private final HardwareMap hardwareMap;
    ReverseTeamElementPipeline pipeline;
    OpenCvWebcam cam;

    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        pipeline = new ReverseTeamElementPipeline(false);
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam_1"));

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
    public void init() {
    }

    public void start() {
    }

    public int detectElement() {
        int target = 0;
        if (pipeline.result != ReverseTeamElementPipeline.Result.Unknown) {
            target = 3;
            if (pipeline.result == ReverseTeamElementPipeline.Result.Left) {
                target = 1;
            } else if (pipeline.result == ReverseTeamElementPipeline.Result.Middle) {
                target = 2;
            } else if (pipeline.result == ReverseTeamElementPipeline.Result.Right) {
                target = 3;
            }
            cam.stopStreaming();
        }
        return target;
    }

}
