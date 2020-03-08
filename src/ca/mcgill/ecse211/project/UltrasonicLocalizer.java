package ca.mcgill.ecse211.project;

import lejos.hardware.Button;

/**
 * This class calibrates the odometer using the ultrasonic sensor.
 * 
 * @author Yudi Xie 260712639
 * @author Zumana Alam 260772572
 */

public class UltrasonicLocalizer extends Thread {

  /**
   * Wall detection threshold in centimeters.
   */
  private static final int distance = 29;

  /**
   * Detection band width in centimeters.
   */
  private static final int k = 4;

  private static final int FACING_WALL = 100;

  private static final int NOT_FACING_WALL = 3;
  public static final double WHEEL_RADIUS = Main.WHEEL_RADIUS;
  public static final double TRACK = Main.TRACK;

  /**
   * Motor speed in degrees per seconds.
   */
  private static final int BASESPEED = 77;

  private static final int ERROR_TO_WALL = 15;

  private static final double DIST_BACK = 7;
  private Odometer odo;
  private UltrasonicPoller us;

  public UltrasonicLocalizer(Odometer odo, UltrasonicPoller us) {
    this.odo = odo;
    this.us = us;
  }

  /*
   * Falling edge method works by detecting the distance between two walls and calculate the angle between two angles
   * and the robot is not facing the walls.
   */
  public void fallingEdge() {
    double theta_right = 0, theta_to45 = 0;
    Main.leftMotor.setSpeed(BASESPEED);
    Main.rightMotor.setSpeed(BASESPEED);

    while (us.getDistance() < FACING_WALL) {
      Main.leftMotor.forward();
      Main.rightMotor.backward();
    }

    // keep checking if it detects a wall
    while (true) {

      Main.leftMotor.backward();
      Main.rightMotor.forward();

      if (underTreshold()) {
        Main.leftMotor.stop(true);
        Main.rightMotor.stop(false);
        break;
      }
    }

    odo.setTheta(0);

    try {
      Main.leftMotor.forward();
      Main.rightMotor.backward();
      UltrasonicPoller.sleep(2000);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }

    while (true) {

      Main.leftMotor.forward();
      Main.rightMotor.backward();

      if (underTreshold()) {
        Main.leftMotor.stop(true);
        Main.rightMotor.stop(false);
        theta_right = odo.getTheta();
        break;
      }
    }

    // calculate the correct angle, and turn to it.
    theta_to45 = 0.5 * theta_right;

    Main.leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta_to45), true);
    Main.rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta_to45), false);

    Main.leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, Math.PI / 4), true);
    Main.rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, Math.PI / 4), false);

    Main.leftMotor.stop(true);
    Main.leftMotor.stop(false);

    odo.setTheta(0);

    try {
      Thread.sleep(20);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

  }

  /*
   * Rising edge method works by detecting the distance between two walls and calculate the angle between two angles and
   * the robot is facing the walls.
   */
  public void risingEdge() {
    double theta_left = 0, theta_to225 = 0;
    Main.leftMotor.setSpeed(BASESPEED);
    Main.rightMotor.setSpeed(BASESPEED);

    while (us.getDistance() > NOT_FACING_WALL) {
      Main.leftMotor.backward();
      Main.rightMotor.forward();
    }

    // keep checking if it detects a wall
    while (true) {

      Main.leftMotor.forward();
      Main.rightMotor.backward();

      if (aboveTheshold()) {
        Main.leftMotor.stop(true);
        Main.rightMotor.stop(false);
        break;
      }
    }

    odo.setTheta(0);

    try {
      Main.leftMotor.backward();
      Main.rightMotor.forward();
      UltrasonicPoller.sleep(2000);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }

    while (true) {

      Main.leftMotor.backward();
      Main.rightMotor.forward();

      if (aboveTheshold()) {
        Main.leftMotor.stop(true);
        Main.rightMotor.stop(false);
        theta_left = odo.getTheta();
        break;
      }
    }

    // calculate the correct angle, and turn to it.
    theta_to225 = 0.5 * theta_left;

    Main.leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, theta_to225), true);
    Main.rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, theta_to225), false);

    Main.leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, Math.toRadians(135)), true);
    Main.rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, Math.toRadians(135)), false);

    Main.leftMotor.stop(true);
    Main.leftMotor.stop(false);
    odo.setTheta(0);

    try {
      Thread.sleep(20);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

  }



  public void run() {

    if (Main.buttonChoice == Button.ID_LEFT) {
      fallingEdge();
    } else if (Main.buttonChoice == Button.ID_RIGHT) {
      risingEdge();
    }

  }

  // check the treshold for the wall detection.
  private boolean underTreshold() {
    return (us.getDistance() < distance - k);
  }

  private boolean aboveTheshold() {
    return (us.getDistance() > distance + k);
  }


  /*
   * This method check the distance between the robot and the wall on both X axis and Y axis, if it is too close to the
   * wall, then the robot will go back for a certain distance.
   */
  public void checkDistance() {

    boolean isGoback = false; // flag for the go back action.

    Main.leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, Math.PI / 2), true);
    Main.rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, Math.PI / 2), false);
    try {
      Main.leftMotor.stop(true);
      Main.rightMotor.stop(false);
      Thread.sleep(200);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }


    if (us.getDistance() < ERROR_TO_WALL) {
      Main.leftMotor.rotate(-convertDistance(WHEEL_RADIUS, DIST_BACK), true);
      Main.rightMotor.rotate(-convertDistance(WHEEL_RADIUS, DIST_BACK), false);
      isGoback = true;
      try {
        Main.leftMotor.stop(true);
        Main.rightMotor.stop(false);
        Thread.sleep(200);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    try {
      Main.leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, Math.PI / 2), true);
      Main.rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, Math.PI / 2), false);
      Main.leftMotor.stop(true);
      Main.rightMotor.stop(false);
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    if (us.getDistance() < ERROR_TO_WALL) {
      Main.leftMotor.rotate(-convertDistance(WHEEL_RADIUS, DIST_BACK), true);
      Main.rightMotor.rotate(-convertDistance(WHEEL_RADIUS, DIST_BACK), false);
      isGoback = true;
      Main.leftMotor.stop(true);
      Main.rightMotor.stop(false);

    }

    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    if (isGoback) {
      Main.leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, Math.PI - 0.1047), true);
      Main.rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, Math.PI - 0.1047), false);
    } else {
      Main.leftMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, Math.PI), true);
      Main.rightMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, Math.PI), false);
    }
    Main.leftMotor.stop(true);
    Main.rightMotor.stop(false);
  }

  // convert the distance and angle
  private static int convertDistance(double radius, double distance) {
    return (int) ((180 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, width * angle / 2);
  }

}
