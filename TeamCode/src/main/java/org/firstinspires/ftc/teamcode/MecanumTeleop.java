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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum: Teleop", group="Mecanum")
//@Disabled
public class MecanumTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum();
    double clawPosition = robot.CLAW_HOME; //Servo's position
    final double CLAW_SPEED = 0.10; // Sets rate to move servo

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.left_stick_x;
            double x = gamepad1.right_stick_x * 0.5;
            double a = (gamepad2.right_trigger * 0.5);

            // Output the safe vales to the motor drives.
            robot.frontLeftDrive.setPower(y + x + rx);
            robot.backLeftDrive.setPower(y - x + rx);
            robot.frontRightDrive.setPower(y - x - rx);
            robot.backRightDrive.setPower(y + x - rx);

            // Y and A control the claw
            if (gamepad2.a) // if the "a" button is pressed on the gamepad, do this next line of code
                clawPosition += CLAW_SPEED; // add to the servo position so it moves
            else if (gamepad2.y) // if the "y" button is pressed, then do the next line of code
                clawPosition -= CLAW_SPEED; // subtract from the servo position so it moves the other direction

            // Move servo to the new position
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE); // make sure the position is valid
            robot.clawServo.setPosition(clawPosition); // this code here ACTUALLY sets the position of the servo so it moves.

            //Arm motor code
            if (gamepad2.right_bumper)
                robot.arm.setPower(1);
            else if (gamepad2.left_bumper)
                robot.arm.setPower(-1);

            //Airplane launcher code

            robot.leftAirplane.setPower(a);
            robot.rightAirplane.setPower(a);

            // Send telemetry message to signify robot running;
            telemetry.addData("y",  "%.2f", y);
            telemetry.addData("rx",  "%.2f", rx);
            telemetry.addData("x",  "%.2f", x);
            telemetry.addData("a",  "%.2f", a);
            telemetry.addData("claw", "%.2f", clawPosition); // VERY IMPORTANT CODE, shows the values on the phone of the servo
            telemetry.addData("arm","%.2f");
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
