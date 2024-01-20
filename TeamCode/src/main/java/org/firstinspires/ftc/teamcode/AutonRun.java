package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="auton run", group="Iterative Opmode")
public class AutonRun extends LinearOpMode {

    //  CHECK THESE MEASUREMENTS!!
    static final double FEET_PER_METER = 3.28084;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //initializing hardware motors and servos:
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //uncomment if we decide to do the backdrop
    private DcMotor pully;

    //  IS IT POSSIBLE TO ADJUST THIS WITH THE ALLIANCE?
    boolean isBackStage = false;

    private Servo SallyTheRClaw;
    boolean isOpenR = false;

    private Servo SallyTheLClaw;
    boolean isOpenL = false;

    private Servo tilt;
    boolean isTilt = false;

    //do i need this:
    boolean low_sens = false;

    //do i need this:
    private final double MAX_CAR_POWER = .5;

    private ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    //initializing state

    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;

    public void HardwareStart() {

        //hardware map:
        telemetry.addData("Status", "Initializing");

        // DRIVING
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Claw initializing
        SallyTheRClaw= hardwareMap.get(Servo.class, "Rclaw");
        SallyTheLClaw= hardwareMap.get(Servo.class, "Lclaw");

        //tilt servo initializing
        tilt = hardwareMap.get(Servo.class, "tilt");

        //Pully initializing
        pully = hardwareMap.get(DcMotor.class, "lift");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //grouping driving motors together
        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        //copied this from henry's hardwareStart method - ask him what he wanted to do with this?
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }

    public void runOpMode() {
        HardwareStart();

        //selecting alliance:
        telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
        telemetry.update();

        String curAlliance = "";
        if (gamepad1.x) {
            curAlliance = "blue";
        } else if (gamepad1.b) {
            curAlliance = "red";
        }
        //SET IF YOU ARE BACK STAGE OR FRONT STAGE HERE WITH THE BOOLEAN VARIABLE
        // if () {isBackStage = }
        // else if () {isBackStage = }

        telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            element_zone = teamElementDetection.elementDetection(telemetry);

            //CHECK IF THESE MATCH UP WITH THE ZONES (MAYBE 1 AND 3 ARE SWITCHED)
            if(element_zone == 1){
                //zone 1
                encoderDrive(4.0, .75, 20.0, "Forward");
                encoderDrive(4.0, 0.75, 12.0, "Left");
                 //THIS IS AN ESTIMATION (TEST TO SEE WHAT THE POWER SHOULD ACTUALLY BE)
                SallyTheRClaw. setPosition(1);

            }
            else if(element_zone == 2){
                //zone 2
                encoderDrive(4.0, .75, 15.0, "Forward");

                SallyTheRClaw. setPosition(1);
            }
            else if(element_zone == 3){
                //zone 3
                encoderDrive(4.0, .75, 15.0, "Forward");
                encoderDrive(4.0, .5, 12.0, "Right");
                SallyTheRClaw. setPosition(1);
            }

            //CAN ADD THE DROP AND PARK?
            //first add a wait function for the amount it takes to do the above
            //to drop the pixel:
            //if box: lift with lift motor and tilt the box with servo
            //if claw: open claw using servo
        }

        //telling user that robot is waiting to start
        telemetry.addData("Object", "Passed & waiting for start");
        telemetry.update();

    }

    public void encoderDrive(double timeout, double power, double dist, String direc) {
        int target = frontLeft.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        if (direc.equals("Forward") || direc.equals("Backward")) {
            frontLeft.setTargetPosition(target);
            backLeft.setTargetPosition(target);
            frontRight.setTargetPosition(target);
            backRight.setTargetPosition(target);
            //Center.setTargetPosition(target);

        }
        if (direc.equals("Left")) {
            frontLeft.setTargetPosition(-target);
            backLeft.setTargetPosition(target);
            frontRight.setTargetPosition(target);
            backRight.setTargetPosition(-target);
            //Center.setTargetPosition(target);
        }
        if (direc.equals("Right")) {
            frontLeft.setTargetPosition(target);
            backLeft.setTargetPosition(-target);
            frontRight.setTargetPosition(-target);
            backRight.setTargetPosition(target);
        }
        runtime.reset();

        if (direc.equals("Forward") || direc.equals("Backward") || direc.equals("Left") || direc.equals("Right")) {
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            while (frontLeft.isBusy()
                    && frontRight.isBusy()
                    && backLeft.isBusy()
                    && backRight.isBusy()
                    && !(runtime.seconds() > timeout)) {
            }
        }
    }
        public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
            //robot stops moving
            motorlf.setPower(0.0);
            motorrf.setPower(0.0);
            motorlb.setPower(0.0);
            motorrb.setPower(0.0);
        }

}
