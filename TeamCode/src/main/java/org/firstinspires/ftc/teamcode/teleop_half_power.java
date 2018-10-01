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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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


@TeleOp(name="Half Power", group="Competition")
//@Disabled
public class teleop_half_power extends LinearOpMode {
    static HardwareClawbot robot       = new HardwareClawbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double    clawOffset  = 0.0 ;                  // Servo mid position
    double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE Bhen the driver hits INIT
     */

    // Declare OpMode members.
    static ElapsedTime runtime = new ElapsedTime();
    /*
     * Code to run ONCE Bhen the driver hits INIT
     */
    //@Override
    //public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
    //    robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
    //    telemetry.addData("Say", "Ready to Rumble");    //
    //}

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
       //runtime.reset();

        // run until the end of the match (driver presses STOP)
        double fLeft;
        double fRight;
        double bLeft;
        double bRight;
        double liftUp;

        while (opModeIsActive()) {
            robot.arm.setPosition(.25);

            fRight = gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            bRight = gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            fLeft = gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            bLeft = gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
            liftUp = gamepad1.right_trigger;


            robot.frontleftDrive.setPower(fLeft / 2);
            robot.frontrightDrive.setPower(fRight / 2);
            robot.backleftDrive.setPower(bLeft / 2);
            robot.backrightDrive.setPower(bRight / 2);
            robot.liftDrive.setPower(liftUp);

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;
            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);


            // Use gamepad buttons to move the arm up (Y) and down (A)
            if (gamepad1.y) {
                robot.arm.setPosition(robot.MID_SERVO + .5);
            }
            else if (gamepad1.a) {
                robot.arm.setPosition(robot.MID_SERVO - .5);
            }
            //if (gamepad1.left_trigger > 0) {
            //    robot.liftDrive_down();
            //}



            //else
            //    robot.arm.setPosition(robot.MID_SERVO - .5);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("front left",  "%.2f", fLeft);
            telemetry.addData("front right", "%.2f", fRight);
            telemetry.addData("back left",  "%.2f", bLeft);
            telemetry.addData("back right", "%.2f", bRight);
            telemetry.addData("Path0",  "Starting at fl%7d fr%7d bl%7d br%7d",
                    robot.frontleftDrive.getCurrentPosition(),
                    robot.frontrightDrive.getCurrentPosition(),
                    robot.backleftDrive.getCurrentPosition(),
                    robot.backrightDrive.getCurrentPosition());
            telemetry.update();

        }
        /* leftDrive.setPower(0);
        rightDrive.setPower(0);
        */
    }
}

