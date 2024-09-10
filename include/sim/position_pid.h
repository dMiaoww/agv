#pragma once

class PositionPID {
public:
  PositionPID(double p, double i, double d, double t)
      : kp(p), ki(i), kd(d), time(t) {}

  double getOutput(double set, double measure, double max = 10) {
    double e = set - measure;
    eall += e;
    double u = kp * (e) + ki * eall + kd * (e - e1) / time;
    u = u > (out + max ) ? (out + max) : u;
    u = u < (out - max ) ? (out - max) : u;
    out = u;
    e1 = e;
    return u;
  }

private:
  double kp, ki, kd;
  double eall = 0;
  double e1 = 0;
  double out = 0;
  double time;
};