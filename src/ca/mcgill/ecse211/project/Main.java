package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class implements the main finite state machine of the code.
 * 
 * @author Yudi Xie 260712639
 * @author Zumana Alam 260772572
 * @author Mathieu Bissonnette 260867497
 * @author Ismail Ziouti
 * @author Mario Yao
 * @author Celine Nader
 */

public class Main {

  /**
   * The left motor port.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The right motor port.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The left launcher motor port.
   */
  public static final EV3LargeRegulatedMotor leftLauncherMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * The right launcher motor port.
   */
  public static final EV3LargeRegulatedMotor rightLauncherMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  /**
   * The ultrasonic sensor port.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));

  /**
   * The color sensor port.
   */
  public static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

  /**
   * The shooter instance.
   */
  public static final Launcher shooter = new Launcher(leftLauncherMotor, rightLauncherMotor);

  /**
   * Holds the user's mode choice.
   */
  public static int buttonChoice;

  /**
   * Radius of the wheel in centimeters.
   */
  public static final double WHEEL_RADIUS = 2.1;

  /**
   * Track width in centimeters.
   */
  public static final double TRACK = 11.4;

  /**
   * Main control logic method.
   * 
   * @param args Command line arguments.
   */

  public static void main(String[] args) {
    /**
     * The EV3 text LCD screen.
     */
    final TextLCD lcd = LocalEV3.get().getTextLCD();

    /*
     * The odometer thread.
     */
    Odometer odometer = new Odometer(leftMotor, rightMotor);

    /*
     * The odometry display thread.
     */
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, lcd);

    /*
     * The ultrasonic poller thread.
     */
    UltrasonicPoller usPoller = new UltrasonicPoller(usSensor);

    /*
     * The ultrasonic localiser object.
     */
    UltrasonicLocalizer usloc = new UltrasonicLocalizer(odometer, usPoller);

    /*
     * The navigation class.
     */
    Navigation navig = new Navigation(odometer, leftMotor, rightMotor);

    /*
     * The light localizer.
     */
    LightLocalizer lightLoc = new LightLocalizer(odometer, usloc, navig);

    // this raise the arm to a angle the make the localization possible
    changeArm(90, 100);

    do {

      // choose between falling edge and rising edge
      lcd.clear();
      lcd.drawString("   Localization   ", 0, 0);
      lcd.drawString(" Falling |  >>>>  ", 0, 1);
      lcd.drawString("  Edge   | Rising ", 0, 2);
      lcd.drawString("  <<<<   | Edge   ", 0, 3);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_DOWN);

    if (buttonChoice == Button.ID_LEFT) {

      // run with falling edge
      lcd.clear();
      lcd.drawString("  FALLING EDGE  ", 0, 3);
      usPoller.start();
      odometer.start();
      odometryDisplay.start();
      usloc.start();

      // wait for another button press to continue with light localizer
      Button.waitForAnyPress();
      lightLoc.start();

      try {
        lightLoc.join();
      } catch (InterruptedException e1) {
        e1.printStackTrace();
      }

      // wait to let the launcher ready
      try {
        Thread.sleep(3000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      shootBall();

    } else if (buttonChoice == Button.ID_RIGHT) {
      // run with risign edge
      lcd.clear();
      lcd.drawString("  RISING EDGE  ", 0, 3);
      usPoller.start();
      odometer.start();
      odometryDisplay.start();
      usloc.start();

      // wait for another button press to continue with light localizer
      Button.waitForAnyPress();
      lightLoc.start();

    } else if (buttonChoice == Button.ID_DOWN) {
      shootBall();
    }

    // Exit if the escape button is pressed.
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }

  /**
   * Move the launcher arm and change its speed.
   * 
   * @param deg Launcher arm angle in degree.
   * @param speed Launcher arm speed in degrees per seconds.
   */
  public static void changeArm(int deg, int speed) {
    leftLauncherMotor.setSpeed(speed);
    rightLauncherMotor.setSpeed(speed);

    leftLauncherMotor.rotate(deg, true);
    rightLauncherMotor.rotate(deg, false);
  }


  /**
   * Calculate the location of a shooting position.
   * 
   * @param x Target's x coordinate in grid position.
   * @param y Target's y coordinate in grid position.
   * @return Array containing the shooting position.
   */
  public static double[] calculateShootingLocation(double x, double y) {
    double[] result = new double[3];

    // calculate the launching position
    if (x < 3.0 && y <= 3.0) {
      result[0] = x + 4.0;
      result[1] = y;
      result[2] = (3.0 / 2.0) * Math.PI;
    } else if (x >= 3.0 && y < 3.0) {
      if (x == 3.0) {
        result[0] = 3.0;
        result[1] = y + 4.0;
        result[2] = Math.PI;
      } else {
        result[0] = x - 4.0;
        result[1] = y;
        result[2] = (1.0 / 2.0) * Math.PI;
      }
    } else if (x > 3.0 && y >= 3.0) {
      result[0] = x - 4.0;
      result[1] = y;
      result[2] = (1.0 / 2.0) * Math.PI;
    } else if (x <= 3.0 && y > 3.0) {
      result[0] = x;
      result[1] = y - 4.0;
      result[2] = 0;
    }
    return result;
  }

  /**
   * Prepare the arm for launching the ball, launch the ball and restore arm position.
   */
  public static void shootBall() {
    changeArm(-40, Launcher.TURNBACK_SPEED);
    try {
      Thread.sleep(1500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    while (true) {
      shooter.shoot();
      try {
        Thread.sleep(3000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      shooter.returnArm();
      int buttonID2 = Button.waitForAnyPress();
      if (buttonID2 == Button.ID_ESCAPE) {
        System.exit(0);
      } else {
        continue;
      }
    }
  }

}

