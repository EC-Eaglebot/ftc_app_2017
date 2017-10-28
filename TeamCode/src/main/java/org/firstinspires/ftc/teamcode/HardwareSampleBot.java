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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareSampleBot
{
    /* Public OpMode members. */
    public DcMotor  frontleftDrive   = null;
    public DcMotor  frontrightDrive  = null;
    public DcMotor  backleftDrive   = null;
    public DcMotor  backrightDrive  = null;
    //public DcMotor  leftArm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSampleBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontleftDrive  = hwMap.get(DcMotor.class, "frontleft_drive");
        frontrightDrive = hwMap.get(DcMotor.class, "frontright_drive");
        backleftDrive  = hwMap.get(DcMotor.class, "backleft_drive");
        backrightDrive = hwMap.get(DcMotor.class, "backright_drive");
       // leftArm    = hwMap.get(DcMotor.class, "left_arm");
        frontleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
<<<<<<< HEAD


=======
>>>>>>> 79056c1f1fadc9e646a087cb098a790844e530bc
    }

    // Function "forward"
    // make robot move forward at specified speed
    void forward(double speed) {
        frontleftDrive.setPower(speed);
        frontrightDrive.setPower(speed);
        backrightDrive.setPower(speed);
        backleftDrive.setPower(speed);
    }
<<<<<<< HEAD
    void go_in_a_circle(double speed) {
        frontleftDrive.setPower(speed);
        frontrightDrive.setPower(-speed);
        backleftDrive.setPower(speed);
        backrightDrive.setPower(-speed);
    }
=======
    // Function "stopMoving"
    // make the movement motors on the robot turn off
>>>>>>> f248d3b26856b792e8f799ac248789f786d11c4f
    void stopMoving() {
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
        backleftDrive.setPower(0);
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
    }
<<<<<<< HEAD
    void backward(double speed){
        frontleftDrive.setPower(speed);
        frontrightDrive.setPower(speed);
        backleftDrive.setPower(speed);
        backleftDrive.setPower(speed);
    }
}
=======
    // Function "turnDegree"
    // make the robot turn a specified degree
    void turnDegree(double degree, ElapsedTime runtime){
        double speed = frontrightDrive.getPower();
        double endTime = (degree / speed) + runtime.seconds();
        // since angle = angular velocity * time,
        // time = angle / velocity
        if (degree > 0) {
            frontrightDrive.setPower(-speed);
            backrightDrive.setPower(-speed);
        } else if (degree < 0) {
            frontleftDrive.setPower(-speed);
            backleftDrive.setPower(-speed);
        }

        while (runtime.seconds() < endTime) {
        } // let the runtime go

        if (degree > 0) {
            frontrightDrive.setPower(speed);
            backrightDrive.setPower(speed);
        } else if (degree < 0) {
            frontleftDrive.setPower(speed);
            backleftDrive.setPower(speed);
        }
    }

    void turnAround(ElapsedTime runtime) {
        turnDegree(180, runtime);
    }
    void turnRight(ElapsedTime runtime){
        turnDegree(90, runtime);
    }
    void turnleft(ElapsedTime runtime){
        turnDegree(-90, runtime);
    }

    void stopAndWait(double secondsToWait, ElapsedTime runtime) {
        stopMoving();
        double endTime = runtime.seconds() + secondsToWait;
        while (runtime.seconds() < endTime) { }
    }
 }
>>>>>>> f248d3b26856b792e8f799ac248789f786d11c4f

