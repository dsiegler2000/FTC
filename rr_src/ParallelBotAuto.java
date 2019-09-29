package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Timer;
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

public abstract class ParallelBotAuto extends BaseAutonomous {

    // Left, right, center

    public ParallelBotAuto(boolean isBlue){

        super(isBlue);

    }

    @Override
    public void runOpMode(){

        try {

            // Run the setup code
            super.runOpMode();
            
            
            /*
            boolean leftJammed = false;
            boolean rightJammed = false;
            
            ElapsedTime timer = new ElapsedTime();
            
            ElapsedTime leftTimer = new ElapsedTime();
            ElapsedTime rightTimer = new ElapsedTime();
            
            while(opModeIsActive() && timer.milliseconds() < 6000) {
    
                int leftSpeed = 700;
                int rightSpeed = 700;
                
                if(leftJammed || rightJammed){
                    
                    robot.setPower(0.2f);
                    
                }
                
                else{
                
                    robot.setPower(-0.3f);
                
                }
                
                if(leftJammed){
                        
                        leftSpeed *= -1;
                        
                        if(leftTimer.milliseconds() > 400){
                            
                            leftJammed = false;
                            
                        }
                        
                    }
                    
                    if(rightJammed){
                        
                        rightSpeed *= -1;
                        
                        if(rightTimer.milliseconds() > 400){
                            
                            rightJammed = false;
                            
                        }
                        
                    }
                
                rightHarvesterMotorEx.setVelocity(rightSpeed, AngleUnit.DEGREES);
                leftHarvesterMotorEx.setVelocity(leftSpeed, AngleUnit.DEGREES);
                
                if(rightHarvesterMotorEx.getVelocity(AngleUnit.DEGREES) < 250 && !rightJammed && rightTimer.milliseconds() > 1200){
                    
                    rightJammed = true;
                    rightTimer.reset();
                    
                }
                
                if(leftHarvesterMotorEx.getVelocity(AngleUnit.DEGREES) < 250 && leftTimer.milliseconds() > 1200 && !leftJammed){
                    
                    leftJammed = true;
                    leftTimer.reset();
                    
                }
                
                telemetry.addData("right", rightHarvesterMotorEx.getVelocity(AngleUnit.DEGREES));
                telemetry.addData("left", leftHarvesterMotorEx.getVelocity(AngleUnit.DEGREES));

                telemetry.update();
    
            }
            
            */
            
            // Bring up the relic flipper
            robot.partiallyRetractRelicFlipper();

            // Bring in the servo and activate Vuforia
            relicTrackables.activate();

            // Take a reading of the pictographs
            countVisibleImages();

            // Knock the jewel
            knockJewel();

            // Initialize the gyro (set the current angle to be 0)
            imu.initialize(gyroParameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Record this starting angle
            float startingAngle = angles.firstAngle;

            // Flip the servo out to get a better view if not on the blue side
            if(!isBlue){
    
                // robot.camcorderServo.setPosition(0.45f);
            
            }

            float motorPower = 0.3f;

            // Mode forward while taking two more readings of the pictographs
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 200, 300, (isBlue ? -1f : 1f) * motorPower);
            countVisibleImages();
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 200, 300, (isBlue ? -1f : 1f) * motorPower);
            countVisibleImages();
            robot.setBrakes(true); // Engage the brakes
            moveForwardUsingEncoder((isBlue ? -1 : 1) * (isBlue ? 950 : 1100), 3000, (isBlue ? -1f : 1f) * motorPower);

            // Bring up the ball knocker if it is limp
            robot.bringUpBallKnocker();

            // Determine which image it was
            int visibleImage = calculateVisibleImage();
            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();

            // Turn 90˚
            gyroTurn(0.13f, startingAngle, 83, 3000);

            // Set the correct strafe power and encoder ticks
            float strafePower = 0.5f * (visibleImage == 1 ? 1 : -1);
            int encoderTicks = 0;

            if(visibleImage == 1){
                
                encoderTicks = -320;
                
                if(!isBlue){
                    
                    encoderTicks = -300;
                    
                }
                
            }
            
            if(visibleImage == 0){
                
                encoderTicks = 280;
                
                if(!isBlue){
                    
                    encoderTicks = 315;
                    
                }
                
            }

            // In case it went limp
            robot.bringUpBallKnocker();

            // Strafe to the correct column
            moveUsingEncoder(encoderTicks, 1400, -strafePower, strafePower, strafePower, -strafePower);

            // Move forward and hit the cryptobox
            moveForwardUsingEncoder(700, 1000, 0.35f);

            // Record the squared up angle against the cryptobox
            float squaredAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Back up a little bit
            moveForwardUsingEncoder(-120, 1000, -0.3f);

            // Just in case it went limp, don't want to damage it
            robot.partiallyRetractRelicFlipper();

            // Flip
            robot.flipperMotor.setPower(-0.8f);
            breakingSleep(1100);
            robot.flipperMotor.setPower(1f);

            // In case it went limp
            robot.bringUpBallKnocker();

            // Back up a bit
            moveForwardUsingEncoder(-180, 2000, -0.5f);

            // In case it went limp
            robot.partiallyRetractRelicFlipper();

            // Flip again
            robot.flipperMotor.setPower(-0.8f);
            breakingSleep(1000);
            robot.flipperMotor.setPower(1f);
            breakingSleep(200);
            
            // moveForwardUsingEncoder(300, 2000, 0.5f);
            // moveForwardUsingEncoder(-150, 2000, -0.5f);

            // Square up with the cryptobox
            float turnPower = 0.3f;
            gyroSquareUp(turnPower, squaredAngle, 3000);
        
            // Start the harvester
            robot.setHarvesterPower(0.7f);

            // Move into the glyph pit
            moveForwardUsingEncoder(-500, 2000, -0.5f);
            // moveForwardUsingEncoder(-420, 2000, -0.25f); replaced with this code
            
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setTargetPosition(-320);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
            robot.setPower(-0.3f);
    
            ElapsedTime time = new ElapsedTime();
            time.reset();
            
            ElapsedTime collectingTimer = new ElapsedTime();
            int state = 0; // 0: both in, 1: right in, left out, 2: left in, right out

            while(opModeIsActive() && robot.frontRight.isBusy() && time.milliseconds() < 2000){
                
                if((state == 0 && collectingTimer.milliseconds() > 300) || (state != 0 && collectingTimer.milliseconds() > 150)){
                    
                    state = state % 2;
                    
                }
                
                if(state == 0){
                    
                    rightHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    
                }
                
                else if(state == 1){
                    
                    rightHarvesterMotorEx.setVelocity(-700, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    
                }
                
                else if(state == 2){
                    
                    rightHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(-700, AngleUnit.DEGREES);
                    
                }
                
                telemetry.addData("FR encoder", robot.frontRight.getCurrentPosition());
                telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
                telemetry.update();
                
            }
    
            robot.setPower(0f);
            
            moveForwardUsingEncoder(300, 1500, 0.5f);
            
            // Turn a little bit
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turnPower = 0.35f;
            breakingSleep(10);
            robot.frontRight.setPower(turnPower);
            robot.frontLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            breakingSleep(80);
            robot.setPower(0f);

            // Do a second trip into the glyph pit
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setTargetPosition(-400);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
            robot.setPower(-0.3f);
            
            time.reset();
            
            collectingTimer = new ElapsedTime();
            state = 0; // 0: both in, 1: right in, left out, 2: left in, right out

            while(opModeIsActive() && robot.frontRight.isBusy() && time.milliseconds() < 2000){
                
                if(collectingTimer.milliseconds() > 220){
                    
                    state = state % 2;
                    
                }
                
                if(state == 0){
                    
                    rightHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    
                }
                
                else if(state == 1){
                    
                    rightHarvesterMotorEx.setVelocity(-700, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    
                }
                
                else if(state == 2){
                    
                    rightHarvesterMotorEx.setVelocity(700, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(-700, AngleUnit.DEGREES);
                    
                }
                
                telemetry.addData("FR encoder", robot.frontRight.getCurrentPosition());
                telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
                telemetry.update();
                
            }
    
            robot.setPower(0f);
            
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.setHarvesterPower(0.65f);
            breakingSleep(100);
            robot.leftHarvesterMotor.setPower(0f);
            breakingSleep(300);
            robot.setHarvesterPower(0.7f);

            // Square up with the cryptobox angle
            turnPower = 0.2f;
            gyroSquareUp(turnPower, squaredAngle, 800);

            // Turn a bit if going for the right or left column
            if(visibleImage == 1){
                
                turnPower = 0.3f * (isBlue ? -1f : 1f);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                robot.frontRight.setPower(-turnPower);
                robot.frontLeft.setPower(turnPower);
                robot.backRight.setPower(-turnPower);
                robot.backLeft.setPower(turnPower); 
                
                breakingSleep(50);
                robot.setPower(0f);
                
            }
            
            else if(visibleImage == 0){
                
                turnPower = -0.3f * (isBlue ? -1f : 1f);;
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                robot.frontRight.setPower(-turnPower);
                robot.frontLeft.setPower(turnPower);
                robot.backRight.setPower(-turnPower);
                robot.backLeft.setPower(turnPower); 
                
                breakingSleep(50);
                robot.setPower(0f);
                
            }

            // Return to the cryptobox
            moveForwardUsingEncoder(1000, 2000, 0.5f);
            moveForwardUsingEncoder(500, 1000, 0.4f);

            // Harvest a bit more to un-jam cubes
            robot.setHarvesterPower(-0.7f);
            breakingSleep(150);
            robot.setHarvesterPower(0.7f);
            breakingSleep(200);
            robot.setHarvesterPower(0f);

            // Lift the motor up to get rid of 3rd glyph
            robot.liftMotor.setPower(-0.55f);

            // Back up a bit
            moveForwardUsingEncoder(-30, 500, -0.6f);
            robot.setHarvesterPower(0f);
            robot.liftMotor.setPower(0f);
            
            robot.setHarvesterPower(-0.7f);
            breakingSleep(150);
            robot.setHarvesterPower(0.7f);
            breakingSleep(200);
            robot.setHarvesterPower(0f);

            // In case limp
            robot.partiallyRetractRelicFlipper();
            
            // Flip
            robot.flipperMotor.setPower(-1f);
            breakingSleep(1500);
            robot.flipperMotor.setPower(0f);

            /*
            // Push the glyph in
            moveForwardUsingEncoder(-170, 2000, -0.6f);
            moveForwardUsingEncoder(200, 2000, 0.4f);
            moveForwardUsingEncoder(-50, 400, -0.3f);
            

            // Flip the glyph in again
            robot.flipperMotor.setPower(1f);
            robot.setPower(0f);
            robot.setHarvesterPower(0f);
            */
            
            // robot.setHarvesterPower(1f);
            moveForwardUsingEncoder(-100, 2000, -0.5f);
            
            // Bring down ball knocker for parking
            robot.bringDownBallKnocker();
            robot.setHarvesterPower(0f);
    
            breakingSleep(700);
            
        } catch(InterruptedException e){
            
            telemetry.addLine("Autonomous stopped.");
            telemetry.update();
            
        }

    }

}