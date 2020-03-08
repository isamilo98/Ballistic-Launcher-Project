package ca.mcgill.ecse211.project;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Keep track of the location of the robot.
 * 
 */

public class Odometer extends Thread {
  // Constants
  private static final long ODOMETER_PERIOD = 25;

  private double x, y, theta;

  private int leftMotorTachoCount, rightMotorTachoCount;

  private EV3LargeRegulatedMotor leftMotor, rightMotor;


  private Object lock;

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It cannot be accessed
   * externally.
   */
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();
  }

  /**
   * This method is where the logic for the odometer will run.
   */
  public void run() {

    long updateStart, updateEnd;
    leftMotor.flt(true);
    rightMotor.flt(false);

    while (true) {

      updateStart = System.currentTimeMillis();

      int currentTachoLeft = leftMotor.getTachoCount();
      int currentTachoRight = rightMotor.getTachoCount();

      double dL = Math.PI * Main.WHEEL_RADIUS * (currentTachoLeft - leftMotorTachoCount) / 180;
      double dR = Math.PI * Main.WHEEL_RADIUS * (currentTachoRight - rightMotorTachoCount) / 180;

      leftMotorTachoCount = currentTachoLeft;
      rightMotorTachoCount = currentTachoRight;

      double dDist = (dL + dR) * 0.5;
      double dTheta = (dL - dR) / Main.TRACK;

      synchronized (lock) {
        theta += dTheta;
        x += dDist * Math.sin(theta);
        y += dDist * Math.cos(theta);
        theta = (theta + 2 * Math.PI) % (2 * Math.PI);
      }


      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
        }
      }
    }
  }

  /**
   * Returns the Odometer data.
   * <p>
   * Writes the current position and orientation of the robot onto the position array. {@code odoData[0] =
   * x, position[1] = y; position[2] = theta;}
   * 
   * @param position the array to store the odometer data
   * @return the odometer data.
   */
  public void getPosition(double[] position, boolean[] update) {
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta * 180 / Math.PI;
    }
  }

  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }


  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
