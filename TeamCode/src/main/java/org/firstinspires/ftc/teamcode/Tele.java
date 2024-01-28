package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOozzzrp", group="Iterative Opmode")
public class Tele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor lift;

    private DcMotor lift2;

    private DcMotor intake;
    boolean intakeOn = false;

    private Servo tilt;
    boolean isTilt = false;

    private Servo airplane;

    boolean low_sens = false;

    private final double MAX_CAR_POWER = .5;

    private DcMotor[] motors;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // DRIVING
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        /*motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;*/


        //Claw initializing
        /*SallyTheRClaw= hardwareMap.get(Servo.class, "Rclaw");
        SallyTheLClaw= hardwareMap.get(Servo.class, "Lclaw");*/

        //Pully initializing - UNCOMMENT THIS ONCE A 5TH MOTOR IS CONNECTED TO THE CONTROL HUB
        lift = hardwareMap.get(DcMotor.class, "lift");

        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        //intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");


        //tilting servo initialized
        tilt = hardwareMap.get(Servo.class, "tilt");

        //airplane launching servo initialization
        airplane = hardwareMap.get(Servo.class, "airplane");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // DRIVING

        //reduces power of the robot:
        if (gamepad1.b) {
            low_sens = true;
        } else if (gamepad1.a) {
            low_sens = false;
        }


        double drive;
        double strafe;
        //double lift;
        double turn;

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.left_stick_x;
        strafe = -gamepad1.right_stick_x;

        if(low_sens){
            drive/=2;
            turn/=2;
            strafe/=2;

        }

        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);



        /*
        ---WHEEL POWERS---
         */
        //We set the power to wheels based off the values calculated above. We can move
        //in all directions based off of the combinations inputted by the driver.

        frontLeft.setPower(lfDrive);
        backLeft.setPower(lbDrive);
        frontRight.setPower(rfDrive);
        backRight.setPower(rbDrive);

        // lift for the lift:
        lift.setPower((0.5 * ((double) gamepad2.right_trigger - (double) gamepad2.left_trigger)));
        lift2.setPower((0.5 * ((double) gamepad2.right_trigger - (double) gamepad2.left_trigger)));


        //intake motor: might wanna trouble shoot this
        if(gamepad2.y && intakeOn == false){
            intake.setPower(-1.00);
            intakeOn = true;
        }
        if(gamepad2.x && intakeOn == true){
            intake.setPower(0.00);
            intakeOn = false;
        }
        //backwards
        if(gamepad2.b){
            intake.setPower(1.00);
            intakeOn = true;

        }

        /*//intake.setPower(gamepad2. right_trigger);
        if(gamepad1.right_bumper == true){
            intake.setPower(1.00);
        }*/

        if(isTilt == false && gamepad2.x == true){
            tilt.setPosition(1); //FIND THE POSITION THIS HAS TO BE WHEN TESTING
            isTilt = true;
            wait(1);

        }
        if(isTilt == true && gamepad2.x == true){
            tilt.setPosition(0); //FIND THE POSITION THIS HAS TO BE WHEN TESTING
            isTilt = false;

        }

        //launching the airplane with GAMEPAD 2:
        if(gamepad1.left_bumper){
            airplane.setPosition(1); //1 goes opposite direction as -1
        }
        if(gamepad1.right_bumper){
            airplane.setPosition(-1);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        for (DcMotor motor : this.motors) {
            motor.setPower(0);
        }
    }


    public void wait(int time) {
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}

