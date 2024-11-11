package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PID Motor TeleOp with Joystick & Buttons", group="Linear OpMode")
public class PIDMotorTeleOp extends LinearOpMode {
    DcMotor testMotor;

    // PID parameters
    double integral = 0;
    double lastError = 0;
    double Kp = 0.001; // Proportional gain
    double Ki = 0.00001; // Integral gain
    double Kd = 0.00001; // Derivative gain
    double targetPosition = 0; // Target position

    // Tuning and timing parameters
    final double POSITION_TOLERANCE = 10; // Acceptable error (tolerance) for reaching the target
    FtcDashboard dashboard;
    ElapsedTime PIDTimer = new ElapsedTime();

    // User-adjustable controls
    final double POSITION_INCREMENT = 50;  // The amount to increase or decrease the position per button press

    @Override
    public void runOpMode() {
        // Initialize motor
        testMotor = hardwareMap.dcMotor.get("testMotor");
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Ensure motor stops moving when power is set to 0
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder to zero
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Use encoder feedback for PID control

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        // Continuous loop to control the motor
        while (opModeIsActive()) {
            // Button-Controlled Positioning
            if (gamepad1.a) {
                targetPosition = 200;  // Set to a specific target (e.g., 1000 encoder counts)
            }
            if (gamepad1.b) {
                targetPosition = 400;  // Set to a specific target (e.g., -1000 encoder counts)
            }
            if (gamepad1.y) {
                targetPosition += POSITION_INCREMENT;  // Increase position by defined increment
            }
            if (gamepad1.x) {
                targetPosition -= POSITION_INCREMENT;  // Decrease position by defined increment
            }

            // Joystick-Controlled Positioning
            // Left joystick controls the target position dynamically
            targetPosition += gamepad1.left_stick_y * 10;  // Scaling factor (10) can be adjusted based on the desired movement speed

            // Allow user to adjust PID constants with D-pad or other buttons
            if (gamepad1.dpad_up) {
                Kp += 0.001;  // Increase Kp
            }
            if (gamepad1.dpad_down) {
                Kp -= 0.001;  // Decrease Kp
            }
            if (gamepad1.dpad_left) {
                Ki += 0.00001;  // Increase Ki
            }
            if (gamepad1.dpad_right) {
                Ki -= 0.00001;  // Decrease Ki
            }
            if (gamepad1.right_bumper) {
                Kd += 0.00001;  // Increase Kd
            }
            if (gamepad1.left_bumper) {
                Kd -= 0.00001;  // Decrease Kd
            }

            // Call the PID control method to calculate motor power
            double motorPower = pidControl(targetPosition);


            motorPower = Math.max(-1, Math.min(1, motorPower));

            // Set the motor power
            testMotor.setPower(motorPower * 0.05);

            // Optional: Display motor position and PID constants on the dashboard for debugging
            telemetry.addData("Motor Position", testMotor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();

            // Sleep to prevent busy waiting and allow time for feedback
            sleep(50);
        }
    }

    // PID control function
    private double pidControl(double targetPosition) {
        double currentPosition = testMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;
        double deltaTime = PIDTimer.time(); // Time since last call

        if (deltaTime > 0) {
            // Calculate integral and derivative
            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            // PID output
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Update last error and reset timer for next calculation
            lastError = error;
            PIDTimer.reset();

            return output;
        } else {
            // If deltaTime is 0, return 0 to prevent division by zero (for the first loop iteration)
            return 0;
        }
    }

    // This method can be added to check if the motor is near the target and stop PID control when the target is reached
    private boolean isAtTargetPosition(double targetPosition) {
        double currentPosition = testMotor.getCurrentPosition();
        return Math.abs(targetPosition - currentPosition) <= POSITION_TOLERANCE;
    }
}

