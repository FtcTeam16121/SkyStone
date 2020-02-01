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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Junk", group="Autonomo")
//@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class RoboFRED_Junkyard extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lftDrvMtr;
    private DcMotor rgtDrvMtr;
    private CRServo servoReach;
    private Servo servoClaw;

    /*
        Gearbox Output Shaft: 753.2 PPR (188.3 rises of channel A)  (https://www.gobilda.com/content/spec_sheets/5202-0002-0027_spec_sheet.pdf)

        PPR = Pulses Per Rotation

        ~2 pulses per degree

        https://www.gobilda.com/content/spec_sheets/5202-0002-0014_spec_sheet.pdf

        Gearbox Output Shaft: 383.6 PPR (95.9 rises of channel A)

        1.065 pulses per degree

    */

    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    //
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.15;

    // CLAW
    private static final double CLAW_MIN = 0.53;
    private static final double CLAW_GRAB = 0.55;
    private static final double CLAW_MAX = 0.95;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lftDrvMtr  = hardwareMap.get(DcMotor.class, "left_drive");
        rgtDrvMtr = hardwareMap.get(DcMotor.class, "right_drive");
        servoReach = hardwareMap.get( CRServo.class, "servoReach");
        servoClaw = hardwareMap.get( Servo.class, "servoClaw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor
        //  that runs backwards when connected directly to the battery
        lftDrvMtr.setDirection(DcMotor.Direction.FORWARD);
        rgtDrvMtr.setDirection(DcMotor.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        lftDrvMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rgtDrvMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lftDrvMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rgtDrvMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                lftDrvMtr.getCurrentPosition(),
                rgtDrvMtr.getCurrentPosition());
        telemetry.update();

        // init the claw
        servoClaw.setPosition( CLAW_MAX - 0.01 );
        servoClaw.setPosition( CLAW_MAX );


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int scaleSpd;

        telemetry.addData("Claw", "Running");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


        encoderDrive(TURN_SPEED, -11.25, 11.25, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        resetDrvPos();

        sleepyTime( 3 );

        encoderDrive(TURN_SPEED, 11.3, -11.3, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        resetDrvPos();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        resetDrvPos();

        while (opModeIsActive()) {
            idle();
        }
        stop();
    }

    /*
     * Taken from:
     * https://github.com/FIRST-Tech-Challenge/SkyStone/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/PushbotAutoDriveByEncoder_Linear.java
     *
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = lftDrvMtr.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rgtDrvMtr.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            lftDrvMtr.setTargetPosition(newLeftTarget);
            rgtDrvMtr.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            lftDrvMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rgtDrvMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lftDrvMtr.setPower(Math.abs(speed));
            rgtDrvMtr.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lftDrvMtr.isBusy() && rgtDrvMtr.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        lftDrvMtr.getCurrentPosition(),
                        rgtDrvMtr.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lftDrvMtr.setPower(0);
            rgtDrvMtr.setPower(0);

            // Turn off RUN_TO_POSITION
            lftDrvMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rgtDrvMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private void clawGrab ( )
    {

        servoClaw.setPosition( CLAW_GRAB );

    }

    private void clawOpen ( )
    {

        servoClaw.setPosition( CLAW_MAX );

    }

    // resets the drive motor encoders
    private void resetDrvPos()
    {
        lftDrvMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rgtDrvMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // silly method that allows me to make noop
    private static void noop()
    {

    }

    private void sleepyTime (double seconds)
    {
        // pause for 1 sec
        try {
            // thread to sleep for 1000 milliseconds
            Thread.sleep((long)(1000*seconds));
        } catch (Exception e) {
            System.out.println(e);
        }
    }


}
