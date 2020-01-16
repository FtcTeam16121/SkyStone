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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="RoboFRED OpMode", group="Iterative Opmode")
public class RoboFRED_TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // hardware control variables
    //private Gyroscope imu;
    private DcMotor lftDrvMtr;
    private DcMotor rgtDrvMtr;
    private DcMotor liftMtr; // aka BOB
    private Servo servoClaw;
    private CRServo servoReach;


    // initial driver controller state (gamepad1)
    private double prvPwr;


    // limits acceleration to prevent spin out
    private static final double LIMITER = 0.006;

    // max speeds
    private static final double MIN_PWR = 0;
    //static final double MIN_TRN = 0;
    private static final double MAX_PWR = 0.65;
    private static final double MAX_TRN = 0.5;

    // claw
    private double clawPos = 0;
    //speed
    //private static final double CLAW_SPD = 0.95;
    // min/max position
    private static final double CLAW_MIN = 0.53;
    private static final double CLAW_GRAB = 0.56;
    private static final double CLAW_MAX = 0.95;
    // servo degrees to double
    //private static final double SERVO_DG_2_DBL = 0.00555555555555556;


    // lift
    // lift variables
    /*
        Gearbox Output Shaft: 753.2 PPR (188.3 rises of channel A)  (https://www.gobilda.com/content/spec_sheets/5202-0002-0027_spec_sheet.pdf)

        PPR = Pulses Per Rotation

        ~2 pulses per degree


    */
    // right bumper up set distance
    private static final int UP_DIST = 753;
    // left bumper down set distance
    private static final int DWN_DIST = 753;

    // bumper speed
    private static final double BMPR_SPD = 1;

    // the crane arm is too heavy for the motor break,
    // so we apply a little power to keep the keep the crane from falling
    private static final double LIFT_BREAK = 0.16;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
         /*
            START INITIALIZE
         */

        lftDrvMtr = hardwareMap.get(DcMotor.class, "left_drive");
        rgtDrvMtr = hardwareMap.get(DcMotor.class, "right_drive");
        liftMtr = hardwareMap.get(DcMotor.class, "liftMtr");
        servoClaw = hardwareMap.get( Servo.class, "servoClaw");
        servoReach = hardwareMap.get( CRServo.class, "servoReach");

        // set direction of right motor to reverse
        rgtDrvMtr.setDirection( DcMotor.Direction.REVERSE );

        // initial prvPwr value
        prvPwr = 0;

        // initialize the lift motor by turning on the break
        liftMtr.setPower( LIFT_BREAK );

        // init the claw
        servoClaw.setPosition( CLAW_MAX - 0.01 );
        servoClaw.setPosition( CLAW_MAX );

        // init reach
        runtime.reset();
        servoReach.setDirection( DcMotorSimple.Direction.FORWARD );
        do {
            noop();
            servoReach.setPower( 1 );
        } while (!(runtime.milliseconds() > 500));
        servoReach.setPower( 0 );


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // END INITIALIZE
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {

        /*
         *
         * Main loop notes:
         *
         * - Go through loop() as fast as possible.
         *    - Only run robot code if the gamepad control is used.
         *    - The faster loop() runs the smoother the robot runs.
         *    - Keep code as simple and fast as possible.
         * - No loops (while, do, etc.) inside of loop() or any function it calls.
         *    - Anything that stalls loop() may crash the robot controller.
         *    - Loops in loop() will delay other robot functions. Meaning, you can't drive and do
         *      something else at the same time.
         *
         */

        // execute drive controls when analog sticks are not at rest
        if (this.gamepad1.left_stick_y != 0
                || this.gamepad1.right_stick_x != 0)
        {
            drive2();
        }
        else
        {
            // stop the motors when the control sticks are not in use
            lftDrvMtr.setPower( 0 );
            rgtDrvMtr.setPower( 0 );

            prvPwr = 0;
        }

        // execute crane and arm controls
        // operate claw
        if (this.gamepad2.dpad_down
                || this.gamepad2.dpad_up
                || this.gamepad2.dpad_left
                || this.gamepad2.dpad_right)
                //&& servoClaw.getPosition() == clawPos)
        {
            //claw();
            claw2();
        }
        //else
        //{
        //    servoClaw.setPower( 0 );
        //}

        // raise/lower crane arm
        if (this.gamepad2.left_bumper
                || this.gamepad2.right_bumper
                || this.gamepad2.left_trigger != 0
                || this.gamepad2.right_trigger != 0
                || this.gamepad2.x)
        {
            lift();
        }
        else
        {
            // put motor into break speed when not in use
            liftMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
            liftMtr.setPower( LIFT_BREAK );
        }

        // moves claw in and out
        if (this.gamepad2.left_stick_y != 0)
        {
            reach();
        }
        else
        {
            // make sure motor is stopped when not used
            servoReach.setPower( 0 );
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("theClaw", "(%.2f)", clawPos);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        lftDrvMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lftDrvMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lftDrvMtr.setPower( 0 );

        rgtDrvMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rgtDrvMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rgtDrvMtr.setPower( 0 );

        liftMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMtr.setPower( 0 );

        //servoClaw.setPower( 0 );
        servoReach.setPower( 0 );
    }


    private void claw2 ()
    {
        // get gamepad2 D-pad state for down and up
        boolean open = this.gamepad2.dpad_up;
        boolean grab = this.gamepad2.dpad_down;
        boolean drag = this.gamepad2.dpad_left;
        boolean halfNeg = this.gamepad2.dpad_right;

        // move the claw
        if ( open )
        {
            clawPos = CLAW_MAX;

        } else if ( grab )
        {
            clawPos = CLAW_GRAB;

        }
        else if ( drag )
        {
            clawPos = CLAW_MIN;
        }
        else if ( halfNeg )
        {
            clawPos =  CLAW_MIN;
        }

        clawPos = Range.clip( clawPos, CLAW_MIN, CLAW_MAX );
        servoClaw.setPosition( clawPos );

    }



    private void lift ()
    {
        // gamepad2
        // left bumper - down a set distance
        boolean dwnSet = this.gamepad2.left_bumper;
        // right bumper - up a set distance
        boolean upSet = this.gamepad2.right_bumper;

        // left trigger - down freeform analog
        float dwnTrgr = this.gamepad2.left_trigger;
        // right trigger - up freeform analog
        float upTrgr = this.gamepad2.right_trigger;

        // x button turns off lift to go under bridge
        boolean xWing = this.gamepad2.x;

        if (xWing)
        {
            liftMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
            liftMtr.setPower( 0 );
        }
        else if (dwnSet && liftMtr.getCurrentPosition() != 0)
        {
            // set lift motor to run to target encoder position and stop with brakes on.
            liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMtr.setTargetPosition( DWN_DIST );
            liftMtr.setPower( BMPR_SPD );

        }
        else if (upSet)
        {
            // set lift motor to run to target encoder position and stop with brakes on.
            liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMtr.setTargetPosition( UP_DIST );
            liftMtr.setPower( -BMPR_SPD );

        }
        else if (dwnTrgr != 0)
        {
            // set lift motor to run in normal mode.
            liftMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
            liftMtr.setPower( -dwnTrgr );
        }
        else if (upTrgr != 0)
        {
            // set lift motor to run to in normal mode.
            liftMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMtr.setPower( upTrgr );
        }
        else
        {
            liftMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
            liftMtr.setPower( 0 );
        }
    }


    private void reach ()
    {
        double reach = this.gamepad2.left_stick_y;

        if (reach < 0)
        {
            servoReach.setDirection( DcMotorSimple.Direction.FORWARD );
            servoReach.setPower( mkDblPos( reach ) );
        }
        else {
            servoReach.setDirection( DcMotorSimple.Direction.REVERSE );
            servoReach.setPower( reach );
        }
    }


    private void drive2()
    {
        // left Y position = forward and reverse speed
        double pwr = -this.gamepad1.left_stick_y;
        // right X position = turn radius
        double trn = -this.gamepad1.right_stick_x;

        String direction, speed, angle;

        double diff, tmpPwr, lftMtrPwr, rgtMtrPwr = 0;

        // region CONDITIONS
        // direction of travel
        if (pwr < 0)
        {
            direction = "FORWARD";
        }
        else if (pwr > 0)
        {
            direction = "REVERSE";
        }
        else if (trn != 0)
        {
            direction = "SPIN";
        }
        else
        {
            direction = "ERROR";
        }

        // robot accelerating or decelerating
        if ((prvPwr > 0 && pwr < 0)
                || (prvPwr < 0 && pwr > 0 ))
        {
            speed = "STOP";
            pwr = 0;
        }
        else if ((prvPwr <= 0 && pwr < prvPwr)
                    || (prvPwr >= 0 && pwr > prvPwr))
        {
            speed = "ACCEL";
        }
        else if (pwr != 0)
        {
            speed = "DECEL";
        }
        else
        {
            speed = "STOP";
            pwr = 0;
        }

        // turning or going straight
        if (trn < 0)
        {
            angle = "RIGHT";
        }
        else if (trn > 0)
        {
            angle = "LEFT";
        }
        else if (pwr != 0)
        {
            angle = "STRAIGHT";
        }
        else
        {
            angle = "ERROR";
        }
        // endregion CONDITIONS

        // region LIMITER
        tmpPwr = pwr;

        if (!speed.equals( "STOP" ) && !direction.equals( "SPIN"))
        {
            pwr = mkDblPos( pwr);
            trn = mkDblPos( trn );
            prvPwr = mkDblPos( prvPwr );

            diff = mkDblPos (pwr - prvPwr);

            if (diff > LIMITER)
            {

                diff = LIMITER;

            }

            if (diff != 0)
            {
                switch (speed)
                {
                    case "ACCEL":
                        pwr = prvPwr + diff;

                        if (pwr > 1)
                        {
                            pwr = 1;
                        }
                        break;

                    case "DECEL":
                        pwr = prvPwr - diff;

                        if (pwr < -1)
                        {
                            pwr = -1;
                        }
                        break;

                    default:
                        pwr = 0;
                        trn = 0;
                }
            }
        }

        // endregion LIMITER

        // region TURN

        /*
            Driver Station Controller A (Driver) notes:
            Max X,Y value for analog sticks is 1.0. Neutral is 0 (zero).

            left analog stick controls fwd and bck motion.

            left Y up = negative stick value = forward
            left Y down = positive stick value = backward

            right analog stick controls turn angle

            right X right = positive = turn right
            right X left = negative = turn left

            Drive Motor Notes:

            Left and right motors are mounted in opposite directions.
            In order to go forward or back the motor power must be opposites.

        */
        // make motor power decisions
        if (direction.equals( "SPIN"))
        {
            lftMtrPwr = -(trn / 2);
            rgtMtrPwr = trn / 2;
        }
        else if (speed.equals( "STOP"))
        {
            lftMtrPwr = 0;
            rgtMtrPwr = 0;
        }
        else
        {
            // now for the hard part... calculating motor power for drive and turn
            switch (direction)
            {
                case "FORWARD":

                    switch (angle)
                    {
                        case "RIGHT":
                            // left motor pwr > right motor pwr
                            rgtMtrPwr = -Range.clip(pwr, MIN_PWR, MAX_PWR);

                            // left motor based on trn but must be less than right motor pwr
                            lftMtrPwr = (trn / 2) - MAX_TRN;
                            break;

                        case "LEFT":
                            // right motor pwr > left motor pwr
                            lftMtrPwr = -Range.clip(pwr, MIN_PWR, MAX_PWR);

                            // left motor based on trn but must be less than right motor pwr
                            rgtMtrPwr = (trn / 2) - MAX_TRN;
                            break;

                        case "STRAIGHT":
                            lftMtrPwr = -Range.clip(pwr, MIN_PWR, MAX_PWR);
                            rgtMtrPwr = -Range.clip(pwr, MIN_PWR, MAX_PWR);
                            break;

                        default:
                            lftMtrPwr = 0;
                            rgtMtrPwr = 0;

                    } // end switch angle
                    break;

                case "REVERSE":

                    switch (angle)
                    {
                        case "RIGHT":
                            // left motor pwr > right motor pwr
                            lftMtrPwr = Range.clip(pwr, MIN_PWR, MAX_PWR);

                            // left motor based on trn but must be less than right motor pwr
                            rgtMtrPwr = MAX_TRN - (trn / 2);
                            break;

                        case "LEFT":
                            // right motor pwr > left motor pwr
                            rgtMtrPwr = Range.clip(pwr, MIN_PWR, MAX_PWR);

                            // left motor based on trn but must be less than right motor pwr
                            lftMtrPwr = MAX_TRN - (trn / 2);
                            break;

                        case "STRAIGHT":
                            lftMtrPwr = Range.clip(pwr, MIN_PWR, MAX_PWR);
                            rgtMtrPwr = Range.clip(pwr, MIN_PWR, MAX_PWR);
                            break;

                        default:
                            lftMtrPwr = 0;
                            rgtMtrPwr = 0;

                    } // end switch angle
                    break;

                default:
                    lftMtrPwr = 0;
                    rgtMtrPwr = 0;
            } //end switch direction
        } // end if-else


        // endregion TURN

        // Setting the motor power
        lftDrvMtr.setPower( lftMtrPwr );
        rgtDrvMtr.setPower( rgtMtrPwr );

        telemetry.addData("Direction", direction);
        telemetry.addData("Speed", speed);
        telemetry.addData("Angle", angle);
        telemetry.update();

        // calculate prvPwr
        if (tmpPwr < 0)
        {
            prvPwr = -pwr;
        }
        else if (tmpPwr > 0)
        {
            prvPwr = pwr;
        }
        else
        {
            prvPwr = 0;
        }
    }


    // Make a number of type double always positive
    // Input: double
    // Output: double
    private double mkDblPos (double input)
    {
        if (input > 0)
        {
            // double is already positive. return input.
            return input;
        } else
        {
            // make the number positive by multiply input with -1
            return (input * -1);
        }
    } // end mkDblPos

    // Make a number of type double always negative
    // Input: double
    // Output: double
    private double mkDblNeg (double input)
    {
        if (input < 0)
        {
            // double is already negative. return input.
            return input;
        } else
        {
            // make the number negative by multiply input with -1
            return (input * -1);
        }
    } // end mkDblNeg

    // silly method that allows me to make noop
    private static void noop()
    {

    }
}
