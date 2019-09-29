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

public abstract class ParallelBotAutoWithDogeCorner extends BaseAutonomous {

    // Left, right, center

    public ParallelBotAutoWithDogeCorner(boolean color){

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
                float turnPower = 0.3f;
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
            float squaredAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            moveForwardUsingEncoder(-400, 1000, -0.1f);

            // In case limp
            robot.bringUpBallKnocker();
            robot.partiallyRetractRelicFlipper();

            // Flip the glyph in
            robot.flipperMotor.setPower(-0.7f);
            sleep(1300);
            robot.flipperMotor.setPower(1f);

            // Back up a little
            moveForwardUsingEncoder(-400, 1000, -0.5f);

            // In case limp
            robot.partiallyRetractRelicFlipper();

            // Flip again
            robot.flipperMotor.setPower(-0.7f);
            sleep(1220);
            robot.flipperMotor.setPower(0.8f);
            sleep(400);

            // Push the glyph in
            moveForwardUsingEncoder(400, 2000, 0.3f);
            moveForwardUsingEncoder(-250, 2000, -0.3f);
            robot.flipperMotor.setPower(0f);

            // Turn towards the glyph pit
            float turnPower = (isBlue ? -1f : 1f) * -0.35f;
            encoderTicks = 15;
            encoderTicks = (!isBlue && visibleImage == 0 ? 5 : encoderTicks);
            encoderTicks = (isBlue && visibleImage == 1 ? 5 : encoderTicks);

            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 300, turnPower, -turnPower, turnPower, -turnPower);

            // In case limp
            robot.bringUpBallKnocker();

            // Move to the glyph pit
            moveForwardUsingEncoder(-900, 3000, -0.6f);

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Get ready to harvest
            robot.setPower(-0.14f);

            rightHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
            leftHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
            ElapsedTime time = new ElapsedTime();
            time.reset();

            ElapsedTime collectingTimer = new ElapsedTime();
            int state = 0; // 0 = Regular, 3 = Jammed
            int consecutiveTimesSeenGlyph = 0;
            int unjammingMode = 0; // 0 = Left goes unpowered, 1 = Right goes unpowered, 2 = spit out

            // While not timed out and we don't have 3 glyphs
            while(opModeIsActive() && time.milliseconds() < 3000 && consecutiveTimesSeenGlyph < 2){

                telemetry.addData("pos", robot.frontRight.getCurrentPosition());
                telemetry.update();

                if(robot.glyphSensor.getLightDetected() > 0.55f){

                    consecutiveTimesSeenGlyph++;

                }

                else{

                    consecutiveTimesSeenGlyph = 0;

                }

                // Figure out if jammed by checking the encoders
                boolean jammed = rightHarvesterMotorEx.getVelocity(AngleUnit.DEGREES) < 200 && leftHarvesterMotorEx.getVelocity(AngleUnit.DEGREES) < 200;
                // Also include a number of cooldown conditions to account for slow acceleration
                boolean cooldownConditionsMet = time.milliseconds() > 200 && state != 3 && collectingTimer.milliseconds() > 800;

                // If jammed, rotate through the unjamming modes
                if(jammed && cooldownConditionsMet){

                    state = 3;
                    collectingTimer.reset();
                    robot.setPower(0.3f);
                    unjammingMode = (unjammingMode + 1) % 3;

                }

                // If jammed, do the unjamming modes
                if(state == 3){

                    if(unjammingMode == 0){

                        rightHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
                        leftHarvesterMotorEx.setVelocity(0, AngleUnit.DEGREES);

                        if(collectingTimer.milliseconds() > 500){

                            collectingTimer.reset();
                            state = 0;
                            robot.setPower(-0.15f);

                        }

                    }

                    else if(unjammingMode == 1){

                        rightHarvesterMotorEx.setVelocity(0, AngleUnit.DEGREES);
                        leftHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);

                        if(collectingTimer.milliseconds() > 500){

                            collectingTimer.reset();
                            state = 0;
                            robot.setPower(-0.15f);

                        }

                    }

                    else if(unjammingMode == 2){

                        rightHarvesterMotorEx.setVelocity(-900, AngleUnit.DEGREES);
                        leftHarvesterMotorEx.setVelocity(-900, AngleUnit.DEGREES);

                        if(collectingTimer.milliseconds() > 550){

                            collectingTimer.reset();
                            state = 0;
                            robot.setPower(-0.15f);

                        }

                    }

                }

                else{

                    rightHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);

                }

            }

            robot.setPower(0f);
            // Spit out for a little bit
            rightHarvesterMotorEx.setVelocity(-700, AngleUnit.DEGREES);
            leftHarvesterMotorEx.setVelocity(-700, AngleUnit.DEGREES);
            breakingSleep(1100);
            robot.setHarvesterPower(0f);

            // Return
            moveUsingEncoder(20, 200, -0.35f, 0.35f, -0.35f, 0.35f);
            rightHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
            leftHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
            moveForwardUsingEncoder(-robot.frontRight.getCurrentPosition(), 4000, 0.4f);
            robot.setHarvesterPower(0f);

            // Turn back to cryptobox
            turnPower = (isBlue ? -1f : 1f) * -0.35f;
            encoderTicks = 10;

            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 700, turnPower, -turnPower, turnPower, -turnPower);

            // Go back to cryptobox
            moveForwardUsingEncoder(3000, 7000, 0.5f);
            moveForwardUsingEncoder(-100, 3000, -0.3f);

            // Square up
            // gyroSquareUp(0.3f, squaredAngle, 1000);
            // moveForwardUsingEncoder(200, 1000, 0.5f);
            // moveForwardUsingEncoder(-20, 1000, -0.3f);


            // In case limp
            robot.bringUpBallKnocker();

            // In case limp
            robot.partiallyRetractRelicFlipper();

            // Flip in extra glyphs
            robot.flipperMotor.setPower(-1f);
            sleep(1300);
            moveForwardUsingEncoder(-150, 1000, -0.5f);
            robot.flipperMotor.setPower(0.75f);

            // Push it in and back up
            moveForwardUsingEncoder(-150, 1000, -0.5f);
            moveUsingEncoder(300, 500, 0.35f, 0.45f, 0.35f, 0.45f);
            moveForwardUsingEncoder(-90, 500, -0.3f);

        } catch(InterruptedException e){

            telemetry.addLine("Autonomous stopped.");
            telemetry.update();

        }

    }

}
