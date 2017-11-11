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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


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


@TeleOp(name="cassidy_auto", group="TestBot")
@Disabled
public class eaglebot_auto_Cassidy extends LinearOpMode {
    static HardwareSampleBot robot       = new HardwareSampleBot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE Bhen the driver hits INIT
     */

    // Declare OpMode members.
    static ElapsedTime runtime = new ElapsedTime();

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

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

        while (opModeIsActive()) {

            dir direction = GetGlyphDirection();
            if (direction == dir.LEFT)  { telemetry.addData("Placing cube left...",vuMark); }
            if (direction == dir.CENTER){ telemetry.addData("Placing cube center...", vuMark); }
            if (direction == dir.RIGHT) { telemetry.addData("Placing cube right...", vuMark); }
            if (direction == dir.ERROR) { telemetry.addData("Error occurred!", vuMark); }
            PlaceCube(direction);

            /* while (runtime.seconds() < 1) {
                leftDrive.setPower(0.2);
                rightDrive.setPower(0.2);
            }
            leftDrive.setPower(0);
            runtime.reset();


            //pick up cube here
            */


            fRight = gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;
            bRight = gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
            fLeft = gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
            bLeft = gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;

            robot.frontleftDrive.setPower(fLeft);
            robot.frontrightDrive.setPower(fRight);
            robot.backleftDrive.setPower(bLeft);
            robot.backrightDrive.setPower(bRight);


            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            /*while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                //strafe or something
                telemetry.addData("Searching...", vuMark);
                telemetry.update();
            }
            */


                    }
        /* leftDrive.setPower(0);
        rightDrive.setPower(0);
        */

        }

    public dir GetGlyphDirection (){
        // put more of the vuforia stuff here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ae0+l9f/////AAAAGSNVpVpt80F4p61FNmwiQSo+nRXp2HcThjA01Uak76AtdklG5SIElxoWuKmM04feEVJp1w1Bgmwq9ttjFbioiq30D/uRCucs90BxX6mAeMrjpCTWv8ySTyQw8Gse/t0OmnQzYlgMe+YJotsbVkiKWJtylDnXP7Lj621oWCH1CQx1vd6fqZ/CVP3AFj37Br/gxTXoyimhrgef4q0MIV4oo0MMDaRkhYEzfKY7qJcopSMKzsoHDFyjnnecUqDnYZAlU9AA/DI8UtnYJ7MoCnmZKS6xir8p6rTSem5Pm3613mBZc40JzVXWdsbtvbR9mfsG+Id1ZA4+q+to/uJCn8RHeWZRYdf7J3Uj6yPAw5SDk+ge";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        relicTrackables.activate();

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return dir.LEFT;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            return dir.CENTER;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return dir.RIGHT;
        } else {
            return dir.ERROR;
        }
    }
    public void PlaceCube ( dir glyphDirection){
        // change the state based on the input
        switch (glyphDirection) {
            case LEFT:
                // handle placing the cube into left position
                robot.turnDegree(-90, runtime);
                robot.stopMoving();
                break;
            case CENTER:
                // handle placing the cube into the center position
                robot.forward(10);
                robot.stopMoving();

                break;
            case RIGHT:
                // handle palcing the cube into the right position
                robot.turnDegree(90, runtime);
                robot.stopMoving();
                break;
            case ERROR:
                // handle error (should never happen)
                robot.stopMoving();
        }
    }
    enum dir {
        LEFT,
        CENTER,
        RIGHT,
        ERROR
    }
}



