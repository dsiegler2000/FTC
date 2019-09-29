package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="FIELD CENTRIC TEST", group="Tests")
public class FieldCentricTeleop extends OpMode {

    private ParallelBotHardware robot = new ParallelBotHardware();

    // have 0 be the center position
    // negative means its turned clockwise
    // positive means counterclockwise
    // figure out how to reset

    private BNO055IMU.Parameters parameters;

    private BNO055IMU imu;

    @Override
    public void init () {

        robot.init(hardwareMap, false);

        // Set up the gyro
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // See the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Get the IMU (must be done here)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addLine("Inited");
        telemetry.update();

    }

    @Override
    public void start(){

        robot.initMovements();

        robot.parkingBrake.setPosition(0.189f);

        robot.partiallyRetractRelicFlipper();
        robot.openRelicGripper();

        // FIGURE OUT SOME WAY TO DEAL WITH THE NOISE

    }

    @Override
    public void loop(){

        /*
        // EXPO CALCULATIONS
        // Calculate the expo and clamped inputs
        double rsy = gamepad1.right_stick_y;
        boolean rsyPos = rsy > 0;

        double rsx = gamepad1.right_stick_x;
        boolean rsxPos = rsx > 0;

        double lsx = gamepad1.left_stick_x;
        boolean lsxPos = lsx > 0;

        // Apply the expo with everything positive
        rsy = Math.abs(Math.pow(Math.abs(rsy), superSlowMode ? SUPER_SLOW_EXPO : DEFAULT_EXPO));
        rsx = Math.abs(Math.pow(Math.abs(rsx), superSlowMode ? SUPER_SLOW_EXPO : DEFAULT_EXPO));
        lsx = Math.abs(Math.pow(Math.abs(lsx), superSlowMode ? SUPER_SLOW_EXPO : DEFAULT_EXPO));

        // Make sure everything is the right sign
        rsy = (rsyPos ? 1f : -1f) * rsy;
        rsx = (rsxPos ? 1f : -1f) * rsx;
        lsx = (lsxPos ? 1f : -1f) * lsx;

        // Clamp the values
        float min = superSlowMode ? -0.6f : -1f;
        float max = superSlowMode ? 0.6f : 1f;

        rsy = clamp(rsy, min, max);
        rsx = clamp(rsx, min, max);
        lsx = clamp(lsx, min, max);
        */

        // Recalibrate gyro
        if(gamepad1.a){

            imu.initialize(parameters);

        }

        // Gyro code
        double rsy = -gamepad1.right_stick_y;
        double rsx = gamepad1.right_stick_x;
        double lsx = gamepad1.left_stick_x;

        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        double forwardMultiplier = Math.cos(heading);
        double strafingMultiplier = Math.sin(heading);

        float frInputs = 0;
        float flInputs = 0;
        float brInputs = 0;
        float blInputs = 0;

        // Forward and backwards
        frInputs -= rsy * forwardMultiplier;
        flInputs -= rsy * forwardMultiplier;
        brInputs -= rsy * forwardMultiplier;
        blInputs -= rsy * forwardMultiplier;

        frInputs += rsy * strafingMultiplier;
        flInputs -= rsy * strafingMultiplier;
        brInputs -= rsy * strafingMultiplier;
        blInputs += rsy * strafingMultiplier;

        
        // Strafing
        frInputs += rsx * forwardMultiplier;
        flInputs -= rsx * forwardMultiplier;
        brInputs -= rsx * forwardMultiplier;
        blInputs += rsx * forwardMultiplier;
        
        // Forward and backwards
        frInputs += rsx * strafingMultiplier;
        flInputs += rsx * strafingMultiplier;
        brInputs += rsx * strafingMultiplier;
        blInputs += rsx * strafingMultiplier;
        

        // Skid steering
        frInputs -= lsx;
        brInputs -= lsx;
        flInputs += lsx;
        blInputs += lsx;

        // Have regular controls on GP2
        // Forward and backwards
        frInputs += gamepad2.right_stick_y;
        flInputs += gamepad2.right_stick_y;
        brInputs += gamepad2.right_stick_y;
        blInputs += gamepad2.right_stick_y;

        // Strafing
        frInputs += gamepad2.right_stick_x;
        flInputs -= gamepad2.right_stick_x;
        brInputs -= gamepad2.right_stick_x;
        blInputs += gamepad2.right_stick_x;

        // Skid steering
        frInputs -= gamepad2.left_stick_x;
        brInputs -= gamepad2.left_stick_x;
        flInputs += gamepad2.left_stick_x;
        blInputs += gamepad2.left_stick_x;

        robot.frontRight.setPower(frInputs);
        robot.frontLeft.setPower(flInputs);
        robot.backRight.setPower(brInputs);
        robot.backLeft.setPower(blInputs);

        // Make sure to update servo pos every loop
        robot.centerBallKnocker();
        robot.closeRelicGripper();
        robot.partiallyRetractRelicFlipper();
        robot.bringUpBallKnocker();
        robot.openFlipperGripper();
        robot.bringUpBallKnocker();

        telemetry.addData("Current heading (radians)", heading);
        telemetry.addData("Current heading (degrees)", heading * (180f / Math.PI));
        telemetry.addData("Forward multiplier", forwardMultiplier);
        telemetry.addData("Strafing multiplier", strafingMultiplier);

        telemetry.update();

    }

}
