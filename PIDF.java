package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

public class PIDF {

    public static class Weights {
        final float kP;
        final float kI;
        final float kD;
        final float kF;
        final float decay;

        public Weights(float _kP, float _kI, float _kD, float _kF, float _decayRate) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            decay = _decayRate;
        }
    }

    public static class Controller {
        final Weights weights;
        float accumulatedError;
        float prev_error;

        /*public void setWeights(Weights _weights) {
            weights = _weights;
        }*/

        public float loop(float _current_x, float _target_x, float _dt) {
            float x_error = _target_x - _current_x;

            /*if (x_error <= 1e-3) {
                x_error = 0;
            }*/


            accumulatedError *= (float) Math.exp(-weights.decay * _dt);
            accumulatedError += x_error * _dt;

            float derivative = (x_error - prev_error) / _dt;


            prev_error = x_error;

            return (weights.kP * x_error + weights.kI * accumulatedError + weights.kD * derivative + _target_x * weights.kF);
        }

        public Controller(Weights _weights) {
            weights = _weights;


            accumulatedError = 0;
            prev_error = 0;


        }


        public void reset() {
            accumulatedError = 0f;
            prev_error = 0f;
        }
    }
}
