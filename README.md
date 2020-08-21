# vl530x_ESP32
<h1>VL53L0X API I2C wrapper for the esp32</h1>
<p>The wrapper is around the I2C driver provided by Espressif. 
Simply put its the interface between ST's <a href='https://www.st.com/en/embedded-software/stsw-img005.html'>VL53l0x API<a> 
and <a href='https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html'>Espressif I2C Driver</a>.
 When I went looking for such a wrapper I could not find one so I am putting this up as it is. I got as far as getting this solution working for some 
 testing and found that the sensor is to slow for my needs.</p>
<h2>How to put it together</h2>
<p>Get the <a href='https://www.st.com/en/embedded-software/stsw-img005.html'>VL53l0x API<a> replace the platform files with the ones in this repo 
    and make sure all of the includes make sense for your project. I think thats all...its been a while</p>
<h2>The example</h2>
<p>The example is simply a modification of an VL53L0X API example. If you want single reading the take a look at the API's single ranging example. 
    To use the VL53L0X API you must call the 'int_I2C_as_master' function from the wrapper first and then call the VL53L0X API functions.</p>
<h2>What to tweek</h2>
<p>The WrapI2C.c has a hard set timeout. The example is set with the GPIO, speed, ... that I was using.</p>
<h2>The purpose</h2>
<p>Clearly I did not spend much/any time to clean and test this as a stand alone solution. The purpose is just to give a little boost. 
    I wasted a good amount of time searching for a ready made solution and then trouble shooting the wrapper without a scope. Really any  
    problems you run into due to something I forgot should be small ie pointing include paths or a definition</p>

<p>The best of luck</p>
