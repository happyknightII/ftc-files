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

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareMecanum class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Teleop Tester", group="Mecanum")
public class MecanumDriveTester extends OpMode{

    /* Declare OpMode members. */
    HardwareMecanum robot       = new HardwareMecanum(); // use the class created to define a Pushbot's hardware
    int             motorNumber = 0 ;                    // Changes which motor to test
    boolean         testRunning = false;                 //Whether motors should run or not
    double          offset      = 0 ;                    //offset for controllers
    final double    DEAD_ZONE   = 0.05 ;                 // Dead zone for the sticks
    boolean[]       notPressed  = {true, true} ;       // Detects if bumpers are held

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        offset = -gamepad1.left_stick_y;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double powerLeft;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y - offset;
        powerLeft = left * ((left > DEAD_ZONE || left < -DEAD_ZONE) ? 1 : 0);
        // Use gamepad left & right Bumpers to open and close the claw
        if (motorNumber > 6) {
            motorNumber = 0;
        } else if (motorNumber < 0) {
            motorNumber = 6;
        } else if (gamepad1.right_bumper) {
            if (notPressed[0]) {
                motorNumber++;
                notPressed[0] = false;
            }
        } else if (gamepad1.left_bumper) {
            motorNumber--;
            if (notPressed[0]) {
                motorNumber++;
                notPressed[0] = false;
            }
        } else {
            notPressed = new boolean[]{true, true};
        }
        if (testRunning) {
            switch (motorNumber) {
                case 1:
                    robot.leftFrontMotor.setPower(powerLeft);
                case 2:
                    robot.rightFrontMotor.setPower(powerLeft);
                case 3:
                    robot.leftRearMotor.setPower(powerLeft);
                case 4:
                    robot.rightRearMotor.setPower(powerLeft);
                case 5:
                    robot.lift.setPower(powerLeft);
            }
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.

        // Use gamepad buttons to turn test on (Y) and off (A)
        if (gamepad1.y) {
            testRunning = false;
        } else if (gamepad1.a) {
            testRunning = true;
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("Power",  "%.2f", left);
        telemetry.addData("Motor Number",  "%.2f", motorNumber);
        telemetry.addData("Testing",  "%.2f", testRunning);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
