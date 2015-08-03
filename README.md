# Calibration Tool for OpenNI based RGB-D sensors

## Build:

```
mkdir build
cd build
cmake .. -DOPEN_NI_ROOT=/your-path-to-OpenNI2/ 
make -j$(nproc)
```

## Dependency

- OpenCV3.0 <http://opencv.org/> -- REQUIRED  
- OpenNI2 <http://structure.io/openni>  -- REQUIRED  

## How to Calibrate a OpenNI sensor

- Print the pattern in chess.pdf and stick the two page together side by side, making a 7x5 pattern.  
  Don't scale the print, since each rectango should be 38mmx38mm.
- Plug in your OpenNI sensor, run `./CalibCapture` program. When you see the color frame window, use a small piece of glue tape to cover the infrad projector of your sensor, so that you will have less noise on the IR image. Point the sensor to the printed pattern, leave the sensor still, then press `s` on the keyboard to save a capture. Captured frame will be shown on separte windows. 
- **DO NOT MOVE THE SENSOR WHEN YOU CAPTURE THE FRAMES!** (this is because OpenNI2 don't give you access to the color and the IR stream of the sensor at the same time, the program is switching the color stream and IR stream on and off, and this takes time.)
- Take around 40 captures from different viewport. **press `q` to exit**
- Run `./StereoCalib`, the program will autometically calibrate the camera and save calibration results to  
  `intrinsics.yml` and `extrinsics.yml`

## Verify result and generate [InfiniTAM](https://github.com/victorprad/InfiniTAM) readable calib file

Run `./TestCalibResult` to visualize a overlay between the color and the depth frame using the calibration result. press `q` to quite the program and a `Calib_ITM.txt` file will be written to the `build/` folder. This file can be directly read from [InfiniTAM](https://github.com/victorprad/InfiniTAM) 
  
