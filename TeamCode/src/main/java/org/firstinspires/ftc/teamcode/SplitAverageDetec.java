package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.SplitAveragePipeline;

public class SplitAverageDetec {
    OpenCvCamera camera;
    SplitAveragePipeline splitAveragePipeline;
    int camW = 800;
    int camH = 448;

    int zone = 1;

    public SplitAverageDetec(HardwareMap hardwareMap){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        splitAveragePipeline = new SplitAveragePipeline();

        camera.setPipeline(splitAveragePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void setAlliance(String alliance){
        splitAveragePipeline.setAlliancePipe(alliance);
    }

    public int elementDetection(Telemetry telemetry) {
        zone = splitAveragePipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }

    public void toggleAverageZone(){
        splitAveragePipeline.toggleAverageZonePipe();
        if (splitAveragePipeline.get_element_zone() == zone1){
            //ZONE 1 CODE
            encoderDrive(4.0, .75, 20.0, "Forward");
            encoderDrive(4.0, 0.75, 12.0, "Left");
            //machine = new StatMachine(driveState);
            //machine.update();
        } else if (splitAveragePipeline.get_element_zone()  == Zone2){
            //ZONE 2 CODE
            encoderDrive(4.0, .75, 15.0, "Forward");
            encoderDrive(4.0, .5, 12.0, "Left");
            //machine.update();
        } else if (splitAveragePipeline.get_element_zone() == Zone3){
            //ZONE 3 CODE
            encoderDrive(4.0, .75, 15.0, "Forward");
            encoderDrive(4.0, .5, 12.0, "Left");
            //machine.update();
        }
    }
}