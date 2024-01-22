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
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Mecanum: RightTimeAuto", group="Mecanum")
//@Disabled
public class RightTimeAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum();
    private ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        //Drive the robot right
        robot.frontLeftDrive.setPower(0.5);
        robot.frontRightDrive.setPower(-0.5);
        robot.backLeftDrive.setPower(0.5);
        robot.backRightDrive.setPower(-0.5);
        sleep(630); //Stay here in the code for 1 second

        //Turn the robot forward
        robot.frontLeftDrive.setPower(0.5);
        robot.frontRightDrive.setPower(0.5);
        robot.backLeftDrive.setPower(0.5);
        robot.backRightDrive.setPower(0.5);
        sleep(640);

        //Park, stop the robot
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        sleep(1400); //Stay here in the code for 1 second

        //Turn the robot backward
        robot.frontLeftDrive.setPower(-0.5);
        robot.frontRightDrive.setPower(-0.5);
        robot.backLeftDrive.setPower(-0.5);
        robot.backRightDrive.setPower(-0.5);
        sleep(720);

        //Drive the robot right
        robot.frontLeftDrive.setPower(0.5);
        robot.frontRightDrive.setPower(-0.5);
        robot.backLeftDrive.setPower(0.5);
        robot.backRightDrive.setPower(-0.5);
        sleep(1700);

        //Park, stop the robot
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        sleep(1400); //Stay here in the code for 1 second

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
