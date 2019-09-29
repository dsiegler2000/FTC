package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

public abstract class ParallelBotAutoCorner extends BaseAutonomous {

    // Left, right, center

    public ParallelBotAutoCorner(boolean color){

        super(color);

    }

    @Override
    public void runOpMode(){
        
        try{
            
            super.runOpMode();

            // Engage the brakes
            robot.setBrakes(true);

            // Bring up the relic flipper
            robot.partiallyRetractRelicFlipper();

            // Activate Vuforia
            relicTrackables.activate();

            // Take a reading of the pictograph
            countVisibleImages();

            knockJewel();

            float motorPower = 0.3f;

            // Move forward and read the pictograph two more times
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 150, 500, (isBlue ? -1f : 1f) * motorPower);
            countVisibleImages();
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 200, 500, (isBlue ? -1f : 1f) * motorPower);
            countVisibleImages();
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 520, 1000, (isBlue ? -1f : 1f) * motorPower);

            // Calculate the visible image
            int visibleImage = calculateVisibleImage();
            
            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();
            
            robot.setPower(0f);
            
            // If its blue, do a 180 turn
            if(isBlue){

                // Init the gyro
                imu.initialize(gyroParameters);

                // Get the starting angle
                float turnPower = 0.13f;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                float startingAngle = angles.firstAngle;

                // Turn
                gyroTurn(turnPower, startingAngle, 176, 6500);

            }

            // In case limp
            robot.bringUpBallKnocker();

            // Set the encoder ticks and power
            float strafePower = -0.3f;
            int encoderTicks = 0;
            
            if(visibleImage == 1){

                encoderTicks = (isBlue ? 840 : 345);

            }

            if(visibleImage == 0){

                encoderTicks = (isBlue ? 260 : 990);

            }

            if(visibleImage == 2){

                encoderTicks = (isBlue ? 505 : 710);

            }
            
            strafePower *= (isBlue ? -1 : 1);

            // Stafe to the column
            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 2000, -strafePower, strafePower, strafePower, -strafePower);

            // Move forward then back a little
            moveForwardUsingEncoder(700, 1000, 0.3f);
            moveForwardUsingEncoder(-400, 1000, -0.1f);

            // In case limp
            robot.bringUpBallKnocker();
            robot.partiallyRetractRelicFlipper();

            // Flip the glyph in
            robot.flipperMotor.setPower(-0.7f);
            sleep(1700);
            robot.flipperMotor.setPower(1f);

            // Back up a little
            moveForwardUsingEncoder(-400, 1000, -0.5f);

            // In case limp
            robot.partiallyRetractRelicFlipper();

            // Flip again
            robot.flipperMotor.setPower(-0.7f);
            sleep(1320);
            robot.flipperMotor.setPower(0.8f);
            sleep(1000);

            // Push the glyph in
            moveForwardUsingEncoder(1000, 1000, 0.3f);
            moveForwardUsingEncoder(-100, 1000, -0.3f);

            // Turn towards the glyph pit
            float turnPower = (isBlue ? -1f : 1f) * -0.35f;
            encoderTicks = 130;
            encoderTicks = (!isBlue && visibleImage == 0 ? 13 : encoderTicks);
            encoderTicks = (isBlue && visibleImage == 1 ? 13 : encoderTicks);
            
            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 700, turnPower, -turnPower, turnPower, -turnPower);

            // In case limp
            robot.bringUpBallKnocker();

            // Harvest and enter the glyph pit
            robot.setHarvesterPower(0.7f);
            moveForwardUsingEncoder(-2000, 3000, -0.3f);
            robot.bringUpBallKnocker();
            moveForwardUsingEncoder(2100, 3000, 0.3f);

            // In case limp
            robot.bringUpBallKnocker();

            // Turn a little to correct
            if(!((visibleImage == 0 && !isBlue) || (visibleImage == 1 && isBlue))){
                
                turnPower *= -1f;
                encoderTicks *= -1f;
        
                moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 700, turnPower, -turnPower, turnPower, -turnPower);
                
            }
            
            // In case limp
            robot.partiallyRetractRelicFlipper();

            // Flip in extra glyphs
            robot.flipperMotor.setPower(-1f);
            sleep(1700);
            robot.flipperMotor.setPower(0.75f);
            robot.setHarvesterPower(0f);
            robot.setPower(0f);

            // Push it in and back up
            moveForwardUsingEncoder(-150, 1000, -0.5f);
            moveForwardUsingEncoder(200, 500, 0.4f);
            moveForwardUsingEncoder(-150, 500, -0.3f);
            
        } catch(InterruptedException e){

            telemetry.addLine("Autonomous stopped.");
            telemetry.update();
            
        }

    }

}
