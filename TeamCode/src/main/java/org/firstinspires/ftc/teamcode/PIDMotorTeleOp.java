package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PID Motor TeleOp", group="Linear OpMode")
public class PIDMotorTeleOp extends LinearOpMode {
    DcMotor testMotor;

    double integral = 0;
    double lastError = 0;
    double Kp = 0.1; // Proportional gain
    double Ki = 0.01; // Integral gain
    double Kd = 0.01; // Derivative gain

    FtcDashboard dashboard;
    ElapsedTime PIDTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.dcMotor.get("testMotor");

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        // Continuous loop to control the motor
        while (opModeIsActive()) {
            // Set target position based on joystick input
            double targetPosition = 0; // Scale joystick input to desired position

            // Call the PID control method
            double motorPower = pidControl(targetPosition);

            // Set the motor power
            testMotor.setPower(motorPower);

            // Optional: Display motor position on dashboard for debugging
            telemetry.addData("Motor Position", testMotor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();

            // Small sleep to prevent busy waiting
            sleep(100);
        }
    }

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

            // Update last error and reset timer
            lastError = error;
            PIDTimer.reset();

            return output;
        } else {
            // If deltaTime is 0, return 0 to prevent division by zero
            return 0;
        }
    }
}
