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
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name="Mecanum Teleop", group="Mecanum")
public class MecanumDriveTeleOp extends OpMode{

    /* Declare OpMode members. */
    HardwareMecanum robot       = new HardwareMecanum();    // use the class created to define hardware
    double[]  slowSpeed         = new double[2];            // slow speed for power assignment
    boolean   intake            = false;                    // intake toggle
    boolean   click             = false;                    // toggle bool

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Robot", "Active");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // creates offset for controllers. Defensive programming
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
        // ship terminology used to describe movement and name variables
        double yaw = gamepad1.right_stick_x;
        double sway = gamepad1.left_stick_x;
        double surge = gamepad1.left_stick_y;
        double lift = gamepad2.left_stick_y / slowSpeed[1];
        double extender = gamepad2.right_stick_y / slowSpeed[1];

        // turns on and off slow mode
        if (gamepad1.right_bumper) {
            slowSpeed[0] = 0.25;
        } else if(gamepad1.left_bumper) {
            slowSpeed[0] = 0.5;
        } else if (gamepad1.left_stick_button) {
            slowSpeed[0] = 1;
        }
        if(gamepad2.left_bumper) {
            slowSpeed[1] = 0.5;
        }else if (gamepad2.right_bumper) {
            slowSpeed[1] = 1;
        }

        // intake toggle
        if (gamepad2.x) {
            if (! click) {
                click = true;
                robot.intake1.setPower(intake ? 1:0);
                robot.intake2.setPower(intake ? -1:0);
                intake = !intake;
            }
        } else {
            click = false;
        }

        // Mecanum wheel calculations
        double[] driveTrainPower = new double[] {
                (surge + sway + yaw),
                (surge + sway - yaw),
                (surge - sway + yaw),
                (surge - sway - yaw)
        };
        for (int i = 0; ++ i < driveTrainPower.length;) {
            driveTrainPower[i] *= slowSpeed[0];
        }

        // motor power assignments. Applies slow mode (except claw)
        robot.leftFrontMotor.setPower(driveTrainPower[0]);
        robot.rightFrontMotor.setPower(driveTrainPower[1]);
        robot.leftRearMotor.setPower(driveTrainPower[2]);
        robot.rightRearMotor.setPower(driveTrainPower[3]);
        robot.lift.setPower(lift);
        robot.extender.setPower(extender);
        robot.claw.setPosition(gamepad2.left_trigger);

        // display numbers to driver
        telemetry.addLine("leftFront")
                .addData("Power",  "%.2f", driveTrainPower[0])
                .addData("Encoder ticks:", "%.2f", (double) robot.leftFrontMotor.getCurrentPosition());
        telemetry.addLine("rightFront")
                .addData("Power", "%.2f", driveTrainPower[1])
                .addData("Encoder ticks", "%.2f", (double) robot.rightFrontMotor.getCurrentPosition());
        telemetry.addLine("leftBack")
                .addData("Power",   "%.2f", driveTrainPower[2])
                .addData("Encoder ticks", "%.2f", (double) robot.leftRearMotor.getCurrentPosition());
        telemetry.addLine("rightBack")
                .addData("Power",  "%.2f", driveTrainPower[3])
                .addData("Encoder ticks", "%.2f", (double) robot.rightRearMotor.getCurrentPosition());
        telemetry.addLine("lift")
                .addData("power",       "%.2f", lift)
                .addData("Encoder ticks", "%.2f", (double) robot.lift.getCurrentPosition());
        telemetry.addData("Extender", "%.2f", extender);
        telemetry.addLine("slow speed")
                .addData("Driver 1",   "%.2f", slowSpeed[0])
                .addData("Driver 2", "%.2f", slowSpeed[1]);
        telemetry.addData("intake",   "%.2f", intake ? 1:0);
        telemetry.addData("rightTrigger","%.2f", gamepad1.right_trigger);
        telemetry.addData("leftTrigger","%.2f", gamepad1.left_trigger);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
