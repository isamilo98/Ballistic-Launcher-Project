package ca.mcgill.ecse211.project;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to navigate to a specific location and to control the movement.
 *
 */
public class Navigation extends Thread {

  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;

  /**
   * Constructor for the navigator class.
   * 
   * @param odometer Odometer used for positionning.
   * @param leftMotor Left motor.
   * @param rightMotor Right motor.
   */
  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  // Constants
  private static final int FWD_SPEED = 150;
  private static final int ROTATION_SPEED = 150;
  private static final double WHEEL_R = Main.WHEEL_RADIUS;
  private static final double TRACK = Main.TRACK;
  private static final double PI = Math.PI;

  private static boolean navigating = false;


  /**
   * Travel to the specified coordinates.
   * 
   * @param x X position in grid coordinates.
   * @param y Y position in grid coordinates.
   */
  public void travelTo(double x, double y) {
    x = x * 30.48;
    y = y * 30.48;
    // Reset motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }

    navigating = true;

    // compute the turn angle
    double dX = x - odometer.getX(); // remaining x distance
    double dY = y - odometer.getY(); // remaining y distance
    double turn_angle = Math.atan2(dX, dY);

    // rotate the robot
    leftMotor.setSpeed(ROTATION_SPEED);
    rightMotor.setSpeed(ROTATION_SPEED);
    turnTo(turn_angle);

    double distance = Math.hypot(dX, dY);

    // move to waypoint
    leftMotor.setSpeed(FWD_SPEED);
    rightMotor.setSpeed(FWD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
  }

  /**
   * Turn the robot to a specific angle.
   * 
   * @param theta Angle to turn to in degrees.
   */
  public void turnTo(double theta) {

    double angle = getMinAngle(theta - odometer.getTheta());

    leftMotor.setSpeed(ROTATION_SPEED);
    rightMotor.setSpeed(ROTATION_SPEED);
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }

  /**
   * Find the minimum angle that the robot can turn. Otherwise the robot could make a maximum turn.
   * 
   * @param angle Angle to find the minimum of in degrees.
   */
  public double getMinAngle(double angle) {
    if (angle > PI) { // Pi = 180 degrees
      angle -= 2 * PI;
    } else if (angle < -PI) {
      angle = angle + 2 * PI;
    }
    return angle;
  }

  /**
   * Determine whether the the robot is navigating.
   */
  public boolean isNavigating() {
    return navigating;
  }

  /**
   * Convert the distance and angle.
   * 
   * @param distance Distance to travel in centimeters.
   * @return Angle of rotation necessary in degree.
   */
  private int convertDistance(double distance) {
    return (int) (360 * distance / (2 * PI * WHEEL_R));
  }

  private int convertAngle(double angle) {
    return convertDistance(TRACK * angle / 2);
  }


}
