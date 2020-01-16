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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.*;


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
@Autonomous(name="Jeff", group="Autonomo")
//@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class RoboFRED_Autonimo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lftDrvMtr;
    private DcMotor rgtDrvMtr;
    private CRServo servoReach;

    /*
        Gearbox Output Shaft: 753.2 PPR (188.3 rises of channel A)  (https://www.gobilda.com/content/spec_sheets/5202-0002-0027_spec_sheet.pdf)

        PPR = Pulses Per Rotation

        ~2 pulses per degree

        https://www.gobilda.com/content/spec_sheets/5202-0002-0014_spec_sheet.pdf

        Gearbox Output Shaft: 383.6 PPR (95.9 rises of channel A)

        1.065 pulses per degree

    */


    private static final int ONE_ROTATION = 753;

    private static final double DRV_SPD = 0.3;

    private static final int SCALE_FAC = (int)(1 / DRV_SPD);
    //private static final int SCALE_FAC = (100);

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor
        //  that runs backwards when connected directly to the battery
        lftDrvMtr.setDirection(DcMotor.Direction.FORWARD);
        rgtDrvMtr.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int scaleSpd;

        telemetry.addData("Claw", "Running");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        // extend claw
        servoReach.setDirection( DcMotorSimple.Direction.FORWARD );
        do {
            servoReach.setPower( 1 );

            sleepyTime( 1 );
        } while (!(runtime.seconds() > 15.0));

        servoReach.setPower( 0 );

        telemetry.addData("Claw", "Stopped");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        // resets the encoder position on the drive motors
        resetDrvPos();

        //trnLft( 90 );

        // drive forward
        drvFwd2( 1.4 );

        //sleepyTime( 2 );

        //drvBcK2( 1 );

        // turn left
        // int spn = ONE_ROTATION; //  / SCALE_FAC
        /*
        lftDrvMtr.setTargetPosition(-spn);
        rgtDrvMtr.setTargetPosition(spn);

        lftDrvMtr.setPower(DRV_SPD);
        rgtDrvMtr.setPower(DRV_SPD);
*/
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

    private void drvFwd (int rotations)
    {
        resetDrvPos();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor
        //  that runs backwards when connected directly to the battery
        lftDrvMtr.setDirection( DcMotor.Direction.FORWARD );
        rgtDrvMtr.setDirection( DcMotor.Direction.REVERSE );

        int scaleSpd = ONE_ROTATION * rotations; //  / SCALE_FAC
        lftDrvMtr.setTargetPosition(scaleSpd);
        rgtDrvMtr.setTargetPosition(scaleSpd);

        // make sure encoder is correctly set
        lftDrvMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rgtDrvMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lftDrvMtr.setPower(DRV_SPD);
        rgtDrvMtr.setPower(DRV_SPD);

        resetDrvPos();
    }

    private void drvBckwd (int rotations)
    {
        resetDrvPos();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor
        //  that runs backwards when connected directly to the battery
        lftDrvMtr.setDirection( DcMotor.Direction.REVERSE );
        rgtDrvMtr.setDirection( DcMotor.Direction.FORWARD );

        int scaleSpd = ONE_ROTATION * rotations; //  / SCALE_FAC
        lftDrvMtr.setTargetPosition(scaleSpd);
        rgtDrvMtr.setTargetPosition(scaleSpd);

        // make sure encoder is correctly set
        lftDrvMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rgtDrvMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lftDrvMtr.setPower(DRV_SPD);
        rgtDrvMtr.setPower(DRV_SPD);

        resetDrvPos();

    }

    private void drvFwd2 (double seconds )
    {
        lftDrvMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rgtDrvMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        lftDrvMtr.setDirection( DcMotor.Direction.FORWARD );
        rgtDrvMtr.setDirection( DcMotor.Direction.REVERSE );

        lftDrvMtr.setPower(DRV_SPD);
        rgtDrvMtr.setPower(DRV_SPD);

        sleepyTime( seconds );

        lftDrvMtr.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rgtDrvMtr.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        lftDrvMtr.setPower(0);
        rgtDrvMtr.setPower(0);
    }

    private void drvBcK2 (double seconds )
    {
        lftDrvMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rgtDrvMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        lftDrvMtr.setDirection( DcMotor.Direction.FORWARD );
        rgtDrvMtr.setDirection( DcMotor.Direction.REVERSE );

        lftDrvMtr.setPower(-DRV_SPD);
        rgtDrvMtr.setPower(-DRV_SPD);

        sleepyTime( seconds );

        lftDrvMtr.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rgtDrvMtr.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        lftDrvMtr.setPower(0);
        rgtDrvMtr.setPower(0);
    }


    private void trnLft ( int degree )
    {
        final int TRN = 10;
        final double SPD = 0.2;

        lftDrvMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rgtDrvMtr.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        lftDrvMtr.setDirection( DcMotor.Direction.FORWARD );
        rgtDrvMtr.setDirection( DcMotor.Direction.REVERSE );

        double seconds = degree * TRN / 1000;

        lftDrvMtr.setPower( SPD );
        rgtDrvMtr.setPower( -SPD );

        sleepyTime( seconds );

        lftDrvMtr.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rgtDrvMtr.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        lftDrvMtr.setPower(0);
        rgtDrvMtr.setPower(0);
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
