package com.sevencrayons.compass;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;
import android.view.animation.Animation;
import android.view.animation.RotateAnimation;
import android.widget.ImageView;

public class Compass implements SensorEventListener
{
	private static final String TAG = "Compass";

	private SensorManager sensorManager;
	private Sensor gsensor;
	private Sensor msensor;
	private float[] mGravity = new float[3];
	private float[] mGeomagnetic = new float[3];
	private float azimuth = 0f;
	private float currectAzimuth = 0;

	// compass arrow to rotate
	public ImageView arrowView = null;

	public Compass(Context context)
	{
		sensorManager = (SensorManager) context
				.getSystemService(Context.SENSOR_SERVICE);
		gsensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		msensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
	}

	public void start()
	{
		sensorManager.registerListener(this, gsensor,
				SensorManager.SENSOR_DELAY_GAME);
		sensorManager.registerListener(this, msensor,
				SensorManager.SENSOR_DELAY_GAME);
	}

	public void stop()
	{
		sensorManager.unregisterListener(this);
	}

	private void adjustArrow()
	{
		if (arrowView == null)
		{
			Log.i(TAG, "arrow view is not set");
			return;
		}

		Log.i(TAG, "will set rotation from " + currectAzimuth + " to "
				+ azimuth);

		Animation an = new RotateAnimation(-currectAzimuth, -azimuth,
				Animation.RELATIVE_TO_SELF, 0.5f, Animation.RELATIVE_TO_SELF,
				0.5f);
		currectAzimuth = azimuth;

		an.setDuration(500);
		an.setRepeatCount(0);
		an.setFillAfter(true);

		arrowView.startAnimation(an);
	}

	protected float[] lowPassFilter(float[] oldVals, float[] newVals)
	{
		/**
		 * Time smoothing constant for low-pass filter.
		 * ALPHA affects how drastically the new value affect the current smoothed value.
		 * 0 ≤ ALPHA ≤ 1 ; a smaller value basically means more smoothing
		 * See: http://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
		 * From: http://blog.thomnichols.org/2011/08/smoothing-sensor-data-with-a-low-pass-filter
		 **/
		final float ALPHA = 0.97f;

		if (newVals == null) return oldVals;

		for (int i=0; i<oldVals.length; i++)
		{
			//newVals[i] = newVals[i] + ALPHA * (oldVals[i] - newVals[i]);
			newVals[i] = ALPHA * newVals[i] + (1 - ALPHA) * oldVals[i];
		}

		return newVals;
	}

	/**
	 * Called when there is a new sensor event.
	 * Note that "on changed" is somewhat of a misnomer, as this will also be called
	 * if we have a new reading from a sensor with the exact same sensor values
	 * (but a newer timestamp).
	 * @param event SensorEvent Called when there is a new sensor event.
	 */
	@Override
	public void onSensorChanged(SensorEvent event)
	{

		synchronized (this)
		{
			//final float ALPHA = 0.97f;

			if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
			{

				// Isolate the force of gravity with the low-pass filter

				mGravity = lowPassFilter(event.values,mGravity);

				/*mGravity[0] = ALPHA * mGravity[0] + (1 - ALPHA)
						* event.values[0];
				mGravity[1] = ALPHA * mGravity[1] + (1 - ALPHA)
						* event.values[1];
				mGravity[2] = ALPHA * mGravity[2] + (1 - ALPHA)
						* event.values[2];*/

				// mGravity = event.values;

				//Log.e("grav", Float.toString(mGravity[0]));
			}

			if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
			{
				// mGeomagnetic = event.values;

				mGeomagnetic = lowPassFilter(event.values, mGeomagnetic);

				/*Geomagnetic[0] = ALPHA * mGeomagnetic[0] + (1 - ALPHA)
						* event.values[0];
				mGeomagnetic[1] = ALPHA * mGeomagnetic[1] + (1 - ALPHA)
						* event.values[1];
				mGeomagnetic[2] = ALPHA * mGeomagnetic[2] + (1 - ALPHA)
						* event.values[2];*/

				//Log.e("geo", Float.toString(mGeomagnetic[0]));

			}

			/**
			 * Computes the inclination matrix I as well as the rotation matrix R
			 * transforming a vector from the device coordinate system to the world's
			 * coordinate system which is defined as a direct orthonormal basis, where:
			 *
			 *		X is defined as the vector product Y.Z (It is tangential
			 *			to the ground at the device's current location and roughly points East).
			 *
			 *		Y is tangential to the ground at the device's current location
			 *			and points towards the magnetic North Pole.
			 *
			 *		Z points towards the sky and is perpendicular to the ground.
			 **/
			float R[] = new float[9];
			float I[] = new float[9];
			boolean success = SensorManager.getRotationMatrix(R, I, mGravity,
					mGeomagnetic);
			if (success)
			{
				float orientation[] = new float[3];
				SensorManager.getOrientation(R, orientation);
				// at this point, orientation contains the azimuth, pitch and roll values.
				// Log.d(TAG, "azimuth (rad): " + azimuth);
				azimuth = (float) Math.toDegrees(orientation[0]); // orientation
				azimuth = (azimuth + 360) % 360;
				// Log.d(TAG, "azimuth (deg): " + azimuth);
				adjustArrow();
			}
		}
	}

	/**
	 * Called when the accuracy of the registered sensor has changed. Unlike onSensorChanged(),
	 * this is only called when this accuracy value changes.
	 * @param sensor Sensor
	 * @param accuracy int: The new accuracy of this sensor, one of SensorManager.SENSOR_STATUS_*
	 */
	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy)
	{
	}
}
