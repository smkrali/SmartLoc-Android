package smartloc.indoorlocalization.com;

import android.hardware.SensorEventListener;

/**
 * Created by spider on 7/16/17.
 */

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.os.Bundle;
import android.os.SystemClock;

import java.util.LinkedList;

/**
 * Created by ijrdn2 on 10/07/2015.
 * A raw reading of the latest values, no mathematical computations have been performed with any values
 */
public class SensorInputListener implements SensorEventListener
{
    private boolean StepCounterRead;
    private int stepsTaken; // TODO: Check if these step are counted since the app started or what... I think it's former.
    //int stepTakenOrNot = 0; // Step taken, then 1.0, otherwise 0.0

    private int mCounterSteps;
    private int mPreviousCounterSteps;

    public SensorInputListener()
    {
        StepCounterRead = false;
        stepsTaken = 0;
        /* Reset the initial step counter value, the first event received by the event listener is
            stored in mCounterSteps and used to calculate the total number of steps taken. */
        mCounterSteps = 0;
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent)
    {
        Sensor sensor = sensorEvent.sensor;
        //float[] values = sensorEvent.values;

        if (sensor.getType() == Sensor.TYPE_STEP_COUNTER){

                if (mCounterSteps < 1) {
                    // initial value
                    mCounterSteps = (int) sensorEvent.values[0];
                }
                stepsTaken = (int) sensorEvent.values[0] - mCounterSteps;
        }
        /*
        else if (sensor.getType() == Sensor.TYPE_STEP_DETECTOR) {
            // Step taken, then 1.0, otherwise 0.0
            stepTakenOrNot = value;
        }
        */
    }

    public int getStepTaken(){
        return stepsTaken;
    }
    /*
    public int getStepTakenOrNot(){
        return stepTakenOrNot;
    }
    */

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy)
    {
    }
}
