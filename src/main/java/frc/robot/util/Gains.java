package frc.robot.util;

public class Gains {
  public double kP;
  public double kI;
  public double kD;
  public double kS;
  public double kV;
  public double kA;
  public double kG;

  public Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
  }

  public void updateGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
  }
}
