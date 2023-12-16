

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class AprilTagDetec extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //TAG IDs of custom sleeve(CHOOSE IDs)
    int Zone1 = 1;
    int Zone2 = 2;
    int Zone3 = 3;

    AprilTagDetection tagOfInterest = null;
    //--DRIVE TRAIN MOTORS---
    DcMotor LeftFront;
    DcMotor LeftBack;
    DcMotor RightFront;
    DcMotor RightBack;

    DcMotor LiftMotor;
    Servo Sally;
        /*
    ---SENSORS---
     */

    String side = "red";

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    //private StatMachine machine;

    //DEFINING STATES
    //Use to tell the robot what state we is
    //EncoderState SpiderRibbitFrogState;
    MoveForwardState driveState;
    //MoveSideState leftState;
    //MoveSideState rightState;

    @Override
    public void runOpMode()
    {
        RightFront = hardwareMap.dcMotor.get("Right Front");
        LeftFront = hardwareMap.dcMotor.get("Left Front");
        RightBack = hardwareMap.dcMotor.get("Right Back");
        LeftBack = hardwareMap.dcMotor.get("Left Back");

        LiftMotor = hardwareMap.dcMotor.get("Claw Motor");
        Sally = hardwareMap.servo.get("Claw Servo");
/*
        ---MOTOR DIRECTIONS---
         */
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        /*
        ---GROUPING---
         */
        motors.add(RightFront);
        motors.add(LeftFront);
        motors.add(RightBack);
        motors.add(LeftBack);

        //SpiderRibbitFrogState = new EncoderState(motors, 33, .9, "Forward");

        //SpiderRibbitFrogState.setNextState(null);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Zone1 || tag.id == Zone2 || tag.id == Zone3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        //machine = new StatMachine(SpiderRibb itFrogState);
        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == Zone2){
            //ZONE 2 CODE
            encoderDrive(4.0, .75, 15.0, "Forward");
            //machine = new StatMachine(driveState);
            //machine.update();
        } else if (tagOfInterest.id == Zone1){
            //ZONE 1 CODE
            encoderDrive(4.0, .75, 15.0, "Forward");
            encoderDrive(4.0, .5, 12.0, "Left");
            //machine.update();
        } else if (tagOfInterest.id == Zone3){
            //ZONE 3 CODE
            encoderDrive(4.0, .75, 15.0, "Forward");
            encoderDrive(4.0, .5, 12.0, "Left");
            //machine.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void encoderDrive(double timeout, double power, double dist, String direc) {
        int target = LeftFront.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        if(direc.equals("Forward") || direc.equals("Backward")) {
            LeftFront.setTargetPosition(target);
            LeftBack.setTargetPosition(target);
            RightFront.setTargetPosition(target);
            RightBack.setTargetPosition(target);
            //Center.setTargetPosition(target);
        }
        if(direc.equals("Left")){
            LeftFront.setTargetPosition(-target);
            LeftBack.setTargetPosition(target);
            RightFront.setTargetPosition(target);
            RightBack.setTargetPosition(-target);
            //Center.setTargetPosition(target);
        }
        if( direc.equals("Right")){
            LeftFront.setTargetPosition(target);
            LeftBack.setTargetPosition(-target);
            RightFront.setTargetPosition(-target);
            RightBack.setTargetPosition(target);
        }

        runtime.reset();

        if(direc.equals("Forward") || direc.equals("Backward") || direc.equals("Left") || direc.equals("Right")) {
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftFront.setPower(power);
            LeftBack.setPower(power);
            RightFront.setPower(power);
            RightBack.setPower(power);

            while (LeftFront.isBusy()
                    && RightFront.isBusy()
                    && LeftBack.isBusy()
                    && RightBack.isBusy()
                    && !(runtime.seconds() > timeout) ) {
            }
        }
        /*if (direc.equals("left") || direc.equals("right")) {
            Center.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Center.setPower(power);
            while (Center.isBusy() && !(runtime.seconds() > timeout)) {
            }
            //stop(leftFront, rightFront, leftBack, rightBack, center);
        }*/

        stop(LeftFront, RightFront, LeftBack, RightBack);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }
}
