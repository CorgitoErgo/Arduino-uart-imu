#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H

double process_acceleration(int input, scales sensor_scale )
{
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double output = 0;
  
  output = input;

  if(sensor_scale == scale_2g)        //for +- 2g
  {
    output = output / 16384;
  }
  else if(sensor_scale == scale_4g)   //for +- 4g
  {
    output = output / 8192;
  }
  else if(sensor_scale == scale_8g)   //for +- 8g
  {
    output = output / 4096;
  }
  else if(sensor_scale == scale_16g)  //for +-16g
  {
    output = output / 2048;
  }
  else
  {
    output = 1;
  }

  //output = output*grav; //uncomment for conversion to gravitational acceleration (m/s^2)
  
  return output;
}

double process_angular_velocity(int16_t input, scales sensor_scale )
{
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */
  double output = 0;

  output = input;
  
  if(sensor_scale == scale_250dps)    //for +- 250 dps
  {
    output = output / 131;
  }
  else if(sensor_scale == scale_500dps)    //for +- 500 dps
  {
    output = output / 65.5;
  }
  else if(sensor_scale == scale_1000dps)   //for +- 1000 dps
  {
    output = output / 32.8;
  }
  else if(sensor_scale == scale_2000dps)   //for +- 2000 dps
  {
    output = output / 16.4;
  }
  else{
    output = 1;
  }

  return output;
}

double process_magnetic_flux(int16_t input, double sensitivity)
{
  /*
  To get magnetic flux density in μT (micro Teslas), each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stored in ROM), then multiplied by some number (calibration)
  and then divided by 0.6 .
  (Faced North each axis should output around 31 µT without any metal / walls around
  Note : This manetometer has really low initial calibration tolerance : +- 500 LSB !!!
  Scale of the magnetometer is fixed -> +- 4800 μT.
  */
  return (input*magnetometer_cal*sensitivity)/0.6;
}

#endif // _FUNCTIONS_H Define Guard
