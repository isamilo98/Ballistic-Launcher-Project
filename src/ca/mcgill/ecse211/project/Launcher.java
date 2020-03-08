package ca.mcgill.ecse211.project;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Controls the launcher arm as to throw a ball.
 *
 */

public class Launcher {

  /**
   * Angluar speed of the launch motors when launching in degrees per seconds.
   */
  public static int LAUNCH_SPEED = 740;

  /**
   * Angluar speed of the launch motors when returning in degrees per seconds.
   */
  public static int TURNBACK_SPEED = 100;

  /**
   * Angular acceleration of the launch motors when launching in degrees per seconds squared.
   */
  public static int LAUNCH_ACC = 8000;
  private EV3LargeRegulatedMotor rightLaunchMotor;
  private EV3LargeRegulatedMotor leftLaunchMotor;

  /**
   * Angle of the throw in degrees.
   */
  public static int launchAngle = 90;

  /**
   * Constructor for the launcher.
   * 
   * @param leftMotor The left regulated motor.
   * @param rightMotor The right regulated motor.
   */

  public Launcher(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.rightLaunchMotor = leftMotor;
    this.leftLaunchMotor = rightMotor;
  }

  /**
   * Move the arm to launch the ball.
   */
  public void shoot() {
    leftLaunchMotor.setSpeed(LAUNCH_SPEED);
    rightLaunchMotor.setSpeed(LAUNCH_SPEED);

    leftLaunchMotor.setAcceleration(LAUNCH_ACC);
    rightLaunchMotor.setAcceleration(LAUNCH_ACC);

    leftLaunchMotor.rotate(launchAngle, true);
    rightLaunchMotor.rotate(launchAngle, false);
  }

  /**
   * Return the arm to the starting position.
   */
  public void returnArm() {
    leftLaunchMotor.setSpeed(TURNBACK_SPEED);
    rightLaunchMotor.setSpeed(TURNBACK_SPEED);

    leftLaunchMotor.rotate(-(launchAngle), true);
    rightLaunchMotor.rotate(-(launchAngle), false);
  }
  
}