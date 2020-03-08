package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Corrects the odometer reading using the light sensor.
 *
 */

public class LightLocalizer extends Thread {

  /**
   * Distance from center of rotation to the light sensor.
   */
  private static final double DISTANCE_CENTER_SENSOR = 13.9;
  
  /**
   * Value difference between ground and black line.
   */
  private static final float SENSOR_VALUE = -20;
  
  /**
   * Launching point real coordinates.
   */
  private static double[] launchingPoint = Main.calculateShootingLocation(6.5, 5.5);

  public static final double WHEEL_R = Main.WHEEL_RADIUS;
  public static final double TRACK = Main.TRACK;

  private EV3ColorSensor lightsensor = Main.lightSensor;

  public static boolean isFinished;

  private SampleProvider colorsensor = lightsensor.getMode("Red");
  private float[] colordata = new float[colorsensor.sampleSize()];

  private Odometer odometer;
  private UltrasonicLocalizer ultraloc;
  private Navigation navig;

  private float color;

  private double[] thetaArray = new double[4];

  /**
   * Light localizer constructor.
   * @param odometer Odometer to be corrected.
   * @param ultraloc Ultasonic localizer.
   * @param navig Navigation object.
   */
  public LightLocalizer(Odometer odometer, UltrasonicLocalizer ultraloc, Navigation navig) {
    this.odometer = odometer;
    this.ultraloc = ultraloc;
    this.navig = navig;
  }

  /**
   * Thread run method. TODO Redo without using a thread!
   */
  public void run() {
    lightsensor.setFloodlight(true);

    // reset the status, and check the distance between the wall and the robot.
    odometer.setTheta(0);
    ultraloc.checkDistance();
    odometer.setTheta(0);

    // the light sensor begin to work
    colorsensor.fetchSample(colordata, 0);
    float lastcolor = colordata[0] * 1000;
    int i = 0;

    /*
     * In this part of the method, the robot will rotate around, and use its light sensor to get
     * data of four directions, and store them as array.
     */
    while (i < 4) {
      colorsensor.fetchSample(colordata, 0);
      color = colordata[0] * 1000;

      Main.leftMotor.setSpeed(120);
      Main.rightMotor.setSpeed(120);
      Main.leftMotor.backward();
      Main.rightMotor.forward();

      if (color - lastcolor < SENSOR_VALUE) {
        lastcolor = color;
        Sound.beep();
        thetaArray[i] = odometer.getTheta();
        i++;
      }
    }

    Main.leftMotor.stop(true);
    Main.rightMotor.stop(false);

    /*
     * the robot then set theta to 0, and X and Y to the coordinates calculated by the formula.
     * And then move to the new X Y coordinates.
     */
    navig.turnTo(0);
    odometer.setTheta(0);
    odometer.setX(-DISTANCE_CENTER_SENSOR * Math.cos((thetaArray[2] - thetaArray[0]) / 2));
    odometer.setY(-DISTANCE_CENTER_SENSOR * Math.cos((thetaArray[3] - thetaArray[1]) / 2));
    LCD.drawString("final: " + "(" + odometer.getX() + ")", 0, 3);
    LCD.drawString("final: " + "(" + odometer.getY() + ")", 0, 4);
    navig.travelTo(0, 0);
    double equalRad = odometer.getTheta();
    equalRad = equalRad % (2 * (Math.PI));
    odometer.setTheta(equalRad);
    navig.turnTo(0); // return the robot back to 0 degree is the last step.
    try {
      Thread.sleep(4000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    navig.travelTo(launchingPoint[0], launchingPoint[1]);

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    navig.turnTo(launchingPoint[2]);
  }

}
