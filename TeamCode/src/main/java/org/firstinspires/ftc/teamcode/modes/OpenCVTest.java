package org.firstinspires.ftc.teamcode.modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GripPipeline;
import org.firstinspires.ftc.teamcode.RingPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous()
public class OpenCVTest extends OpMode {

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    int cameraMonitorViewId;
    OpenCvCamera camera;
    GripPipeline pipeline;

    @Override
    public void init() {
        cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new GripPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        FtcDashboard.getInstance().startCameraStream(camera, 30);
    }

    @Override
    public void loop() {
        ArrayList<RingPoint> rings = pipeline.ringsOutput();

        telemetry.addData("Ring found", rings.size());
        if (rings.size() > 0) {
            RingPoint rp = rings.get(0);

            telemetry.addLine("Ring[0]: " + rp.getX() + ", " + rp.getY());
        }

        telemetry.update();
    }
}


