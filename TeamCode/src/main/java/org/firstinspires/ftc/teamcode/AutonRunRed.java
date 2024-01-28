package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="auton run RED", group="Iterative Opmode")
public class AutonRunRed extends LinearOpMode {

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


    private Servo tilt;
    boolean isTilt = false;

    //CHANGE POS AFTER TESTING TO SEE WHAT IT SHOULD ACTUALLY BE CUZ THIS WOULD SPIN 360 DEGREES:
    double pos = 1.00;

    private final double MAX_CAR_POWER = .5;

    private ArrayList<DcMotor> motors = new ArrayList<DcMotor>();


    //initializing state

    public int element_zone;

    private TeamElementSubsystem teamElementDetection = null;

    boolean togglePreview = true;

    String curAlliance = "red";

    public void HardwareStart() {

        //hardware map:
        telemetry.addData("Status", "Initializing");

        // DRIVING */
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        /*frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);*/

        tilt = hardwareMap.get(Servo.class, "tilt");

        //tilt servo initializing
        /*tilt = hardwareMap.get(Servo.class, "tilt");

        //lift initializing
        //lift = hardwareMap.get(DcMotor.class, "lift");

        //grouping driving motors together*/
        motors.add(frontLeft);
        motors.add(frontRight);
        motors.add(backLeft);
        motors.add(backRight);

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();

    }

    public void runOpMode() {
        HardwareStart();

        while (!opModeIsActive() && !isStopRequested()) {

            //SET IF YOU ARE BACK STAGE OR FRONT STAGE HERE WITH THE BOOLEAN VARIABLE

            telemetry.update();
        }

        waitForStart();
        //Telemetry telemetry = new Telemetry();
        element_zone = teamElementDetection.elementDetection();

        //set element_zone to intended zone for testing
        //CHECK IF THESE MATCH UP WITH THE ZONES (MAYBE 1 AND 3 ARE SWITCHED)
        if(element_zone == 1){
            //zone 1 (left zone)
            //distance from middle to the side zone is 13 inch
            encoderDrive(4.0, .75, -20, "Backward");
            encoderDrive(4.0, 0.75, 12.0, "Right");

            //reset position:
            encoderDrive(4.0, 0.75, 5.0, "Forward");

        }
        else if(element_zone == 2){
            //zone 2 (front zone)

                /*  the length from back to the tape line is 47 3/4
                    the length from back to the tape line is 47 3/4 & robot length = 18 inch
                    --> the distance travel length = 47 - 18 = 30 inch
                    */

            encoderDrive(4.0, .75, -30.0, "Backward");

            //reset position:
            encoderDrive(4.0, .75, 5.0, "Forward");


        }
        else if(element_zone == 3){
            //zone 3 (right zone)

            //distance from middle to the side zone is 13 inch
            encoderDrive(4.0, .75, -20.0, "Backward");
            encoderDrive(4.0, .5, 12.0, "Left");

            //to get back to a forward position that wont knock over pixel placed later:
            //want to turn 90 degrees right
            encoderDrive(4.0, .5, 5.0, "Forward");
        }

        //first add a wait function for the amount it takes to do the above? - might not be needed
        //to drop the pixel:
        //if box: lift with lift motor and tilt the box with servo
        //if claw: open claw using servo


        //PLACING ON BACKBOARD:(could add the prev ifs but want to separate)
        //different between different alliances bc it would be mirror - imaged

        //PARKING:
        //version after placing pixel on board:
        //version after placing pixel in zones

        //release 2nd pre-loaded pixel:
        tilt.setPosition(pos);




        //CAN ADD THE DROP AND PARK?
        //first add a wait function for the amount it takes to do the above
        //to drop the pixel:
        //if box: lift with lift motor and tilt the box with servo
        //if claw: open claw using servo

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

    //put this at the end of encoder method
        public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
            //robot stops moving
            motorlf.setPower(0.0);
            motorrf.setPower(0.0);
            motorlb.setPower(0.0);
            motorrb.setPower(0.0);
        }

}
